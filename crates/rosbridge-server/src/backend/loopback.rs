//! In-process backend: a message bus connecting clients of this server without
//! any external middleware. Used by the test-suite and for browser↔browser
//! bridging. Publishers fan out CDR to subscribers; client-hosted services and
//! actions are reachable by client callers.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use async_trait::async_trait;
use parking_lot::Mutex;
use tokio::sync::{mpsc, oneshot};

use super::*;

#[derive(Default)]
struct Inner {
    /// topic -> active subscriber sinks.
    topic_subs: HashMap<String, Vec<(SubscriptionId, mpsc::UnboundedSender<Sample>)>>,
    /// publisher id -> topic.
    publishers: HashMap<PublisherId, String>,
    /// subscription id -> topic (for cleanup).
    sub_topic: HashMap<SubscriptionId, String>,
    /// service name -> request sink.
    services: HashMap<String, mpsc::UnboundedSender<ServiceRequest>>,
    service_names: HashMap<ServiceServerId, String>,
    /// action name -> goal sink.
    actions: HashMap<String, mpsc::UnboundedSender<ActionGoal>>,
    action_names: HashMap<ActionServerId, String>,
    /// (action, goal_id) -> cancel trigger.
    cancels: HashMap<(String, [u8; 16]), oneshot::Sender<()>>,
}

/// An in-process [`RosBackend`].
#[derive(Default)]
pub struct LoopbackBackend {
    inner: Mutex<Inner>,
    counter: AtomicU64,
}

impl LoopbackBackend {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn shared() -> SharedBackend {
        Arc::new(Self::new())
    }

    fn next_id(&self) -> u64 {
        self.counter.fetch_add(1, Ordering::Relaxed)
    }
}

#[async_trait]
impl RosBackend for LoopbackBackend {
    fn advertise(
        &self,
        topic: &str,
        _type_name: &str,
        _qos: &QosSpec,
    ) -> Result<PublisherId, BackendError> {
        let id = PublisherId(self.next_id());
        self.inner.lock().publishers.insert(id, topic.to_string());
        Ok(id)
    }

    fn publish(&self, id: PublisherId, cdr: &[u8]) -> Result<(), BackendError> {
        let mut guard = self.inner.lock();
        let topic = match guard.publishers.get(&id) {
            Some(t) => t.clone(),
            None => return Err(BackendError::Failed("unknown publisher".into())),
        };
        if let Some(subs) = guard.topic_subs.get_mut(&topic) {
            subs.retain(|(_, tx)| {
                tx.send(Sample { cdr: cdr.to_vec() }).is_ok()
            });
        }
        Ok(())
    }

    fn unadvertise(&self, id: PublisherId) {
        self.inner.lock().publishers.remove(&id);
    }

    fn subscribe(
        &self,
        topic: &str,
        _type_name: &str,
        _qos: &QosSpec,
    ) -> Result<(SubscriptionId, mpsc::UnboundedReceiver<Sample>), BackendError> {
        let id = SubscriptionId(self.next_id());
        let (tx, rx) = mpsc::unbounded_channel();
        let mut guard = self.inner.lock();
        guard
            .topic_subs
            .entry(topic.to_string())
            .or_default()
            .push((id, tx));
        guard.sub_topic.insert(id, topic.to_string());
        Ok((id, rx))
    }

    fn unsubscribe(&self, id: SubscriptionId) {
        let mut guard = self.inner.lock();
        if let Some(topic) = guard.sub_topic.remove(&id) {
            if let Some(subs) = guard.topic_subs.get_mut(&topic) {
                subs.retain(|(sid, _)| *sid != id);
            }
        }
    }

    async fn call_service(
        &self,
        service: &str,
        _type_name: &str,
        request_cdr: Cdr,
        timeout_secs: f64,
    ) -> Result<Cdr, BackendError> {
        let sender = self
            .inner
            .lock()
            .services
            .get(service)
            .cloned()
            .ok_or_else(|| BackendError::Failed(format!("service {service} not available")))?;
        let (resp_tx, resp_rx) = oneshot::channel();
        sender
            .send(ServiceRequest {
                request_cdr,
                responder: resp_tx,
            })
            .map_err(|_| BackendError::Failed("service server gone".into()))?;
        let dur = std::time::Duration::from_secs_f64(timeout_secs.max(0.0));
        match tokio::time::timeout(dur, resp_rx).await {
            Ok(Ok(Some(cdr))) => Ok(cdr),
            Ok(Ok(None)) => Err(BackendError::Failed("service returned failure".into())),
            Ok(Err(_)) => Err(BackendError::Failed("service server dropped".into())),
            Err(_) => Err(BackendError::Timeout),
        }
    }

    fn advertise_service(
        &self,
        service: &str,
        _type_name: &str,
    ) -> Result<(ServiceServerId, mpsc::UnboundedReceiver<ServiceRequest>), BackendError> {
        let id = ServiceServerId(self.next_id());
        let (tx, rx) = mpsc::unbounded_channel();
        let mut guard = self.inner.lock();
        guard.services.insert(service.to_string(), tx);
        guard.service_names.insert(id, service.to_string());
        Ok((id, rx))
    }

    fn unadvertise_service(&self, id: ServiceServerId) {
        let mut guard = self.inner.lock();
        if let Some(name) = guard.service_names.remove(&id) {
            guard.services.remove(&name);
        }
    }

    async fn send_action_goal(
        &self,
        action: &str,
        _type_name: &str,
        goal_cdr: Cdr,
    ) -> Result<GoalStream, BackendError> {
        let sender = self
            .inner
            .lock()
            .actions
            .get(action)
            .cloned()
            .ok_or_else(|| BackendError::Failed(format!("action {action} not available")))?;

        let goal_id = self.make_goal_id();
        let (fb_tx, fb_rx) = mpsc::unbounded_channel();
        let (inner_res_tx, inner_res_rx) = oneshot::channel::<(Option<Cdr>, i8)>();
        let (cancel_tx, cancel_rx) = oneshot::channel::<()>();
        let (gs_res_tx, gs_res_rx) = oneshot::channel::<Result<(Cdr, i8), String>>();

        // Bridge the hosting server's (Option<Cdr>, status) into the issuer's
        // Result form.
        tokio::spawn(async move {
            let mapped = match inner_res_rx.await {
                Ok((Some(cdr), status)) => Ok((cdr, status)),
                Ok((None, status)) => Err(format!("action aborted (status {status})")),
                Err(_) => Err("action server dropped".to_string()),
            };
            let _ = gs_res_tx.send(mapped);
        });

        self.inner
            .lock()
            .cancels
            .insert((action.to_string(), goal_id), cancel_tx);

        sender
            .send(ActionGoal {
                goal_cdr,
                goal_id,
                feedback_tx: fb_tx,
                result_tx: inner_res_tx,
                cancel_rx,
            })
            .map_err(|_| BackendError::Failed("action server gone".into()))?;

        Ok(GoalStream {
            feedback: fb_rx,
            result: gs_res_rx,
            goal_id,
        })
    }

    fn cancel_action_goal(&self, action: &str, goal_id: &[u8; 16]) {
        if let Some(tx) = self
            .inner
            .lock()
            .cancels
            .remove(&(action.to_string(), *goal_id))
        {
            let _ = tx.send(());
        }
    }

    fn advertise_action(
        &self,
        action: &str,
        _type_name: &str,
    ) -> Result<(ActionServerId, mpsc::UnboundedReceiver<ActionGoal>), BackendError> {
        let id = ActionServerId(self.next_id());
        let (tx, rx) = mpsc::unbounded_channel();
        let mut guard = self.inner.lock();
        guard.actions.insert(action.to_string(), tx);
        guard.action_names.insert(id, action.to_string());
        Ok((id, rx))
    }

    fn unadvertise_action(&self, id: ActionServerId) {
        let mut guard = self.inner.lock();
        if let Some(name) = guard.action_names.remove(&id) {
            guard.actions.remove(&name);
        }
    }

    fn now(&self) -> (i32, u32) {
        let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        (d.as_secs() as i32, d.subsec_nanos())
    }
}

impl LoopbackBackend {
    fn make_goal_id(&self) -> [u8; 16] {
        let n = self.next_id();
        let t = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let mut id = [0u8; 16];
        id[..8].copy_from_slice(&n.to_le_bytes());
        id[8..].copy_from_slice(&t.to_le_bytes());
        id
    }
}
