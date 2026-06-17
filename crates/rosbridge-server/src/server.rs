//! WebSocket server: accepts connections, runs the rosbridge handshake, and
//! pumps frames between each socket and its [`ClientSession`].

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::Message;

use ros_message::Registry;

use crate::backend::SharedBackend;
use crate::config::SharedConfig;
use crate::session::{ClientSession, OutFrame, Shared};

/// The rosbridge WebSocket server.
pub struct Server {
    shared: Arc<Shared>,
    client_seq: AtomicU64,
    connected: Arc<AtomicU64>,
}

impl Server {
    /// Build a server from configuration, a type registry, and a ROS backend.
    pub fn new(cfg: SharedConfig, registry: Arc<Registry>, backend: SharedBackend) -> Arc<Self> {
        Arc::new(Server {
            shared: Shared::new(cfg, registry, backend),
            client_seq: AtomicU64::new(0),
            connected: Arc::new(AtomicU64::new(0)),
        })
    }

    /// Number of currently connected clients.
    pub fn connected_clients(&self) -> u64 {
        self.connected.load(Ordering::Relaxed)
    }

    /// Bind and serve forever. Returns the bound address via `on_bind` before
    /// entering the accept loop (useful for tests that need the actual port).
    pub async fn serve<F>(self: Arc<Self>, on_bind: F) -> std::io::Result<()>
    where
        F: FnOnce(std::net::SocketAddr),
    {
        let cfg = &self.shared.cfg;
        let host = if cfg.address.is_empty() {
            "0.0.0.0"
        } else {
            cfg.address.as_str()
        };
        let listener = self.bind_with_retry(host, cfg.port).await?;
        let local = listener.local_addr()?;
        tracing::info!("rosbridge_server listening on {local}{}", cfg.url_path);
        on_bind(local);

        loop {
            let (stream, peer) = match listener.accept().await {
                Ok(v) => v,
                Err(e) => {
                    tracing::warn!("accept error: {e}");
                    continue;
                }
            };
            let _ = stream.set_nodelay(true);
            let id = self.client_seq.fetch_add(1, Ordering::Relaxed);
            let me = self.clone();
            tokio::spawn(async move {
                if let Err(e) = me.handle_connection(stream, id).await {
                    tracing::debug!("client {id} ({peer}) ended: {e}");
                }
            });
        }
    }

    /// Bind, retrying on `OSError`-style failures per `retry_startup_delay`.
    async fn bind_with_retry(&self, host: &str, port: u16) -> std::io::Result<TcpListener> {
        let addr = format!("{host}:{port}");
        let delay = Duration::from_secs_f64(self.shared.cfg.retry_startup_delay.max(0.0));
        loop {
            match TcpListener::bind(&addr).await {
                Ok(l) => return Ok(l),
                Err(e) => {
                    if delay.is_zero() {
                        return Err(e);
                    }
                    tracing::warn!("bind {addr} failed ({e}); retrying in {delay:?}");
                    tokio::time::sleep(delay).await;
                }
            }
        }
    }

    // The accept-handshake closure's `Err` type (`ErrorResponse`) is fixed by
    // the tungstenite API, so its size is not something we control here.
    #[allow(clippy::result_large_err)]
    async fn handle_connection(
        self: &Arc<Self>,
        stream: TcpStream,
        id: u64,
    ) -> anyhow::Result<()> {
        let expected_path = self.shared.cfg.url_path.clone();
        let ws = tokio_tungstenite::accept_hdr_async(
            stream,
            move |req: &Request, resp: Response| -> Result<Response, ErrorResponse> {
                // Accept all origins (matches rosbridge check_origin == True);
                // enforce url_path when it is not the root.
                if expected_path != "/" && req.uri().path() != expected_path {
                    let mut err = ErrorResponse::new(Some("path not found".into()));
                    *err.status_mut() =
                        tokio_tungstenite::tungstenite::http::StatusCode::NOT_FOUND;
                    return Err(err);
                }
                Ok(resp)
            },
        )
        .await?;

        let n = self.connected.fetch_add(1, Ordering::Relaxed) + 1;
        tracing::info!("client {id} connected ({n} total)");

        let (mut sink, mut source) = ws.split();
        let (out_tx, mut out_rx) = mpsc::unbounded_channel::<OutFrame>();
        let session = ClientSession::new(id, self.shared.clone(), out_tx.clone());

        // Optional periodic ping.
        let ping_interval = self.shared.cfg.websocket_ping_interval;
        let ping_task = if ping_interval > 0.0 {
            let tx = out_tx.clone();
            Some(tokio::spawn(async move {
                let mut tick =
                    tokio::time::interval(Duration::from_secs_f64(ping_interval));
                loop {
                    tick.tick().await;
                    if tx.send(OutFrame::Ping(Vec::new())).is_err() {
                        break;
                    }
                }
            }))
        } else {
            None
        };

        // Writer task: drain outgoing frames to the socket.
        let delay = self.shared.cfg.delay_between_messages;
        let writer = tokio::spawn(async move {
            while let Some(frame) = out_rx.recv().await {
                let msg = match frame {
                    OutFrame::Text(s) => Message::Text(s),
                    OutFrame::Binary(b) => Message::Binary(b),
                    OutFrame::Ping(d) => Message::Ping(d),
                    OutFrame::Pong(d) => Message::Pong(d),
                    OutFrame::Close => {
                        let _ = sink.send(Message::Close(None)).await;
                        break;
                    }
                };
                if sink.send(msg).await.is_err() {
                    break;
                }
                if delay > 0.0 {
                    tokio::time::sleep(Duration::from_secs_f64(delay)).await;
                }
            }
            let _ = sink.flush().await;
        });

        // Reader loop.
        while let Some(msg) = source.next().await {
            match msg {
                Ok(Message::Text(t)) => session.handle_text(t.as_str()),
                Ok(Message::Binary(b)) => session.handle_binary(&b),
                Ok(Message::Ping(d)) => {
                    let _ = out_tx.send(OutFrame::Pong(d.to_vec()));
                }
                Ok(Message::Pong(_)) | Ok(Message::Frame(_)) => {}
                Ok(Message::Close(_)) => break,
                Err(_) => break,
            }
        }

        // Cleanup.
        session.shutdown();
        let _ = out_tx.send(OutFrame::Close);
        if let Some(p) = ping_task {
            p.abort();
        }
        writer.abort();
        let left = self.connected.fetch_sub(1, Ordering::Relaxed) - 1;
        tracing::info!("client {id} disconnected ({left} remaining)");
        Ok(())
    }
}
