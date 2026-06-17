//! End-to-end tests driving the server over real WebSocket connections with
//! the in-process loopback backend.

use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use ros_message::Registry;
use rosbridge_server::backend::loopback::LoopbackBackend;
use rosbridge_server::config::Config;
use rosbridge_server::server::Server;
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{connect_async, MaybeTlsStream, WebSocketStream};

type Ws = WebSocketStream<MaybeTlsStream<TcpStream>>;

/// Start a server on an ephemeral port and return its `ws://` URL.
async fn start_server() -> String {
    let mut cfg = Config {
        port: 0,
        ..Default::default()
    };
    cfg.finalize_globs();
    let registry = Arc::new(Registry::with_standard_types());
    let backend = Arc::new(LoopbackBackend::new());
    let server = Server::new(Arc::new(cfg), registry, backend);

    let (tx, rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let _ = server
            .serve(move |addr| {
                let _ = tx.send(addr);
            })
            .await;
    });
    let addr = rx.await.expect("server bound");
    format!("ws://127.0.0.1:{}/", addr.port())
}

async fn connect(url: &str) -> Ws {
    let (ws, _) = connect_async(url).await.expect("connect");
    ws
}

async fn send(ws: &mut Ws, v: Value) {
    ws.send(Message::Text(serde_json::to_string(&v).unwrap()))
        .await
        .unwrap();
}

/// Read the next text message as JSON, with a timeout.
async fn next_json(ws: &mut Ws) -> Value {
    loop {
        let msg = tokio::time::timeout(Duration::from_secs(5), ws.next())
            .await
            .expect("timeout waiting for message")
            .expect("stream ended")
            .expect("ws error");
        match msg {
            Message::Text(t) => return serde_json::from_str(&t).unwrap(),
            Message::Binary(b) => return ciborium::from_reader(&b[..]).unwrap(),
            Message::Ping(_) | Message::Pong(_) => continue,
            other => panic!("unexpected frame: {other:?}"),
        }
    }
}

/// Read the next binary frame (e.g. CBOR), with a timeout.
async fn next_binary(ws: &mut Ws) -> Vec<u8> {
    loop {
        let msg = tokio::time::timeout(Duration::from_secs(5), ws.next())
            .await
            .expect("timeout")
            .expect("stream ended")
            .expect("ws error");
        match msg {
            Message::Binary(b) => return b.to_vec(),
            Message::Ping(_) | Message::Pong(_) => continue,
            other => panic!("expected binary, got {other:?}"),
        }
    }
}

#[tokio::test]
async fn pubsub_roundtrip() {
    let url = start_server().await;
    let mut pubr = connect(&url).await;
    let mut subr = connect(&url).await;

    send(
        &mut subr,
        json!({"op":"subscribe","topic":"/chatter","type":"std_msgs/msg/String"}),
    )
    .await;
    // Give the subscription time to register.
    tokio::time::sleep(Duration::from_millis(100)).await;

    send(
        &mut pubr,
        json!({"op":"advertise","topic":"/chatter","type":"std_msgs/msg/String"}),
    )
    .await;
    send(
        &mut pubr,
        json!({"op":"publish","topic":"/chatter","msg":{"data":"hello rust"}}),
    )
    .await;

    let got = next_json(&mut subr).await;
    assert_eq!(got["op"], "publish");
    assert_eq!(got["topic"], "/chatter");
    assert_eq!(got["msg"]["data"], "hello rust");
}

#[tokio::test]
async fn subscribe_type_inferred_from_publisher() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    // Advertise first so the type is learned.
    send(
        &mut a,
        json!({"op":"advertise","topic":"/num","type":"std_msgs/msg/Int32"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    // Subscribe without a type.
    send(&mut b, json!({"op":"subscribe","topic":"/num"})).await;
    tokio::time::sleep(Duration::from_millis(50)).await;

    send(
        &mut a,
        json!({"op":"publish","topic":"/num","msg":{"data":42}}),
    )
    .await;
    let got = next_json(&mut b).await;
    assert_eq!(got["msg"]["data"], 42);
}

#[tokio::test]
async fn cbor_subscription_yields_binary() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    send(
        &mut b,
        json!({"op":"subscribe","topic":"/c","type":"std_msgs/msg/String","compression":"cbor"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(
        &mut a,
        json!({"op":"advertise","topic":"/c","type":"std_msgs/msg/String"}),
    )
    .await;
    send(
        &mut a,
        json!({"op":"publish","topic":"/c","msg":{"data":"cbor!"}}),
    )
    .await;

    let bin = next_binary(&mut b).await;
    let v: Value = ciborium::from_reader(&bin[..]).unwrap();
    assert_eq!(v["op"], "publish");
    assert_eq!(v["msg"]["data"], "cbor!");
}

#[tokio::test]
async fn service_call_roundtrip() {
    let url = start_server().await;
    let mut server_client = connect(&url).await;
    let mut caller = connect(&url).await;

    // server_client hosts /add of type AddTwoInts.
    send(
        &mut server_client,
        json!({"op":"advertise_service","service":"/add","type":"example_interfaces/srv/AddTwoInts"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(100)).await;

    // caller calls it.
    send(
        &mut caller,
        json!({"op":"call_service","service":"/add",
               "type":"example_interfaces/srv/AddTwoInts",
               "args":{"a":3,"b":4},"id":"call1"}),
    )
    .await;

    // server_client receives the forwarded request.
    let req = next_json(&mut server_client).await;
    assert_eq!(req["op"], "call_service");
    assert_eq!(req["args"]["a"], 3);
    let req_id = req["id"].as_str().unwrap().to_string();

    // server_client answers.
    send(
        &mut server_client,
        json!({"op":"service_response","service":"/add","id":req_id,
               "values":{"sum":7},"result":true}),
    )
    .await;

    // caller gets the response.
    let resp = next_json(&mut caller).await;
    assert_eq!(resp["op"], "service_response");
    assert_eq!(resp["result"], true);
    assert_eq!(resp["values"]["sum"], 7);
    assert_eq!(resp["id"], "call1");
}

#[tokio::test]
async fn action_roundtrip_with_feedback() {
    let url = start_server().await;
    let mut server_client = connect(&url).await;
    let mut caller = connect(&url).await;

    send(
        &mut server_client,
        json!({"op":"advertise_action","action":"/fib",
               "type":"example_interfaces/action/Fibonacci"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(100)).await;

    send(
        &mut caller,
        json!({"op":"send_action_goal","action":"/fib",
               "action_type":"example_interfaces/action/Fibonacci",
               "args":{"order":5},"feedback":true,"id":"g1"}),
    )
    .await;

    // Server receives the goal.
    let goal = next_json(&mut server_client).await;
    assert_eq!(goal["op"], "send_action_goal");
    assert_eq!(goal["args"]["order"], 5);
    let goal_id = goal["id"].as_str().unwrap().to_string();

    // Server emits feedback then result.
    send(
        &mut server_client,
        json!({"op":"action_feedback","action":"/fib","id":goal_id,
               "values":{"sequence":[0,1,1]}}),
    )
    .await;
    send(
        &mut server_client,
        json!({"op":"action_result","action":"/fib","id":goal_id,
               "values":{"sequence":[0,1,1,2,3,5]},"status":4,"result":true}),
    )
    .await;

    // Caller observes feedback then result (order preserved).
    let fb = next_json(&mut caller).await;
    assert_eq!(fb["op"], "action_feedback");
    assert_eq!(fb["values"]["sequence"], json!([0, 1, 1]));

    let res = next_json(&mut caller).await;
    assert_eq!(res["op"], "action_result");
    assert_eq!(res["result"], true);
    assert_eq!(res["values"]["sequence"], json!([0, 1, 1, 2, 3, 5]));
    assert_eq!(res["status"], 4);
}

#[tokio::test]
async fn fragmentation_reassembly() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    // Subscribe with a tiny fragment_size to force fragmentation.
    send(
        &mut b,
        json!({"op":"subscribe","topic":"/big","type":"std_msgs/msg/String",
               "fragment_size":20}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(
        &mut a,
        json!({"op":"advertise","topic":"/big","type":"std_msgs/msg/String"}),
    )
    .await;
    let big = "x".repeat(200);
    send(
        &mut a,
        json!({"op":"publish","topic":"/big","msg":{"data": big}}),
    )
    .await;

    // Collect fragments and reassemble.
    let mut parts: Vec<(usize, String)> = Vec::new();
    let mut total;
    loop {
        let v = next_json(&mut b).await;
        assert_eq!(v["op"], "fragment");
        total = v["total"].as_u64().unwrap() as usize;
        parts.push((
            v["num"].as_u64().unwrap() as usize,
            v["data"].as_str().unwrap().to_string(),
        ));
        if parts.len() == total {
            break;
        }
    }
    assert!(total > 1);
    parts.sort_by_key(|(n, _)| *n);
    let joined: String = parts.into_iter().map(|(_, d)| d).collect();
    let reassembled: Value = serde_json::from_str(&joined).unwrap();
    assert_eq!(reassembled["op"], "publish");
    assert_eq!(reassembled["msg"]["data"], "x".repeat(200));
}

/// Try to read a JSON message within `ms`, returning `None` on timeout.
async fn try_next_json(ws: &mut Ws, ms: u64) -> Option<Value> {
    match tokio::time::timeout(Duration::from_millis(ms), ws.next()).await {
        Ok(Some(Ok(Message::Text(t)))) => Some(serde_json::from_str(&t).unwrap()),
        _ => None,
    }
}

#[tokio::test]
async fn set_level_none_suppresses_status() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    // Silence status messages for client b.
    send(&mut b, json!({"op":"set_level","level":"none"})).await;
    // This would normally yield a status error, now suppressed.
    send(&mut b, json!({"op":"bogus"})).await;
    // Subscribe and have a publish to b, which must be the first thing we see.
    send(
        &mut b,
        json!({"op":"subscribe","topic":"/lvl","type":"std_msgs/msg/String"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(
        &mut a,
        json!({"op":"advertise","topic":"/lvl","type":"std_msgs/msg/String"}),
    )
    .await;
    send(
        &mut a,
        json!({"op":"publish","topic":"/lvl","msg":{"data":"hi"}}),
    )
    .await;

    let got = next_json(&mut b).await;
    assert_eq!(got["op"], "publish", "status should have been suppressed");
}

#[tokio::test]
async fn throttle_drops_intermediate_messages() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    send(
        &mut b,
        json!({"op":"subscribe","topic":"/t","type":"std_msgs/msg/Int32",
               "throttle_rate":1000}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(
        &mut a,
        json!({"op":"advertise","topic":"/t","type":"std_msgs/msg/Int32"}),
    )
    .await;
    for i in 0..5 {
        send(&mut a, json!({"op":"publish","topic":"/t","msg":{"data":i}})).await;
    }

    // First message arrives promptly.
    let first = next_json(&mut b).await;
    assert_eq!(first["op"], "publish");
    // No second message within the throttle window.
    assert!(
        try_next_json(&mut b, 300).await.is_none(),
        "throttle should drop intermediate messages"
    );
}

#[tokio::test]
async fn unsubscribe_stops_delivery() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    send(
        &mut b,
        json!({"op":"subscribe","topic":"/u","type":"std_msgs/msg/String","id":"s1"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(&mut b, json!({"op":"unsubscribe","topic":"/u","id":"s1"})).await;
    tokio::time::sleep(Duration::from_millis(50)).await;

    send(
        &mut a,
        json!({"op":"advertise","topic":"/u","type":"std_msgs/msg/String"}),
    )
    .await;
    send(
        &mut a,
        json!({"op":"publish","topic":"/u","msg":{"data":"x"}}),
    )
    .await;

    assert!(
        try_next_json(&mut b, 300).await.is_none(),
        "no delivery after unsubscribe"
    );
}

#[tokio::test]
async fn service_failure_response() {
    let url = start_server().await;
    let mut server_client = connect(&url).await;
    let mut caller = connect(&url).await;

    send(
        &mut server_client,
        json!({"op":"advertise_service","service":"/maybe","type":"std_srvs/srv/SetBool"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    send(
        &mut caller,
        json!({"op":"call_service","service":"/maybe","type":"std_srvs/srv/SetBool",
               "args":{"data":true},"id":"c1"}),
    )
    .await;

    let req = next_json(&mut server_client).await;
    let req_id = req["id"].as_str().unwrap().to_string();
    // Respond with failure.
    send(
        &mut server_client,
        json!({"op":"service_response","service":"/maybe","id":req_id,"result":false}),
    )
    .await;

    let resp = next_json(&mut caller).await;
    assert_eq!(resp["op"], "service_response");
    assert_eq!(resp["result"], false);
}

#[tokio::test]
async fn cbor_binary_publish_to_server_is_accepted() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    let mut b = connect(&url).await;

    send(
        &mut b,
        json!({"op":"subscribe","topic":"/bin","type":"std_msgs/msg/String"}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Send advertise + publish as CBOR binary frames.
    let adv = json!({"op":"advertise","topic":"/bin","type":"std_msgs/msg/String"});
    let pubmsg = json!({"op":"publish","topic":"/bin","msg":{"data":"from cbor"}});
    let mut buf = Vec::new();
    ciborium::into_writer(&adv, &mut buf).unwrap();
    a.send(Message::Binary(buf)).await.unwrap();
    let mut buf2 = Vec::new();
    ciborium::into_writer(&pubmsg, &mut buf2).unwrap();
    a.send(Message::Binary(buf2)).await.unwrap();

    let got = next_json(&mut b).await;
    assert_eq!(got["msg"]["data"], "from cbor");
}

#[tokio::test]
async fn unknown_op_returns_status_error() {
    let url = start_server().await;
    let mut a = connect(&url).await;
    send(&mut a, json!({"op":"bogus_op"})).await;
    let v = next_json(&mut a).await;
    assert_eq!(v["op"], "status");
    assert_eq!(v["level"], "error");
}
