use rosbridge_rust::{
    format_rosbridge_publish, forward_broadcast_to_browser, parse_client_message, ClientOp,
    SubscriptionKey, SubscriptionRegistry,
};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tokio_tungstenite::tungstenite::Message;

#[tokio::test]
async fn test_subscription_registry_lifecycle() {
    let registry = SubscriptionRegistry::new();
    let key1 = SubscriptionKey::new("/battery", Some("sub_vin".to_string()));
    let key2 = SubscriptionKey::new("/charger", None);

    registry
        .register(
            key1.clone(),
            "{\"op\":\"subscribe\",\"topic\":\"/battery\",\"id\":\"sub_vin\"}".to_string(),
        )
        .await;
    registry
        .register(
            key2.clone(),
            "{\"op\":\"subscribe\",\"topic\":\"/charger\"}".to_string(),
        )
        .await;

    let snapshot = registry.snapshot().await;
    assert_eq!(snapshot.len(), 2);

    // Unregister key1
    registry.unregister(&key1).await;
    let snapshot_after = registry.snapshot().await;
    assert_eq!(snapshot_after.len(), 1);
    assert_eq!(
        snapshot_after[0],
        "{\"op\":\"subscribe\",\"topic\":\"/charger\"}"
    );
}

#[tokio::test]
async fn test_async_forward_broadcast_delivers_formatted_messages() {
    let (broadcast_tx, broadcast_rx) = broadcast::channel(16);
    let (out_tx, mut out_rx) = mpsc::channel(16);

    let forward_handle = tokio::spawn(async move {
        forward_broadcast_to_browser(
            broadcast_rx,
            "/robot_pose".to_string(),
            Some("pose_1".to_string()),
            out_tx,
            0,
        )
        .await;
    });

    let payload: Arc<str> = Arc::from("{\"x\":1.5,\"y\":2.5}");
    broadcast_tx.send(payload).unwrap();

    let msg = out_rx.recv().await.unwrap();
    if let Message::Text(text) = msg {
        assert_eq!(
            text,
            "{\"op\":\"publish\",\"topic\":\"/robot_pose\",\"id\":\"pose_1\",\"msg\":{\"x\":1.5,\"y\":2.5}}"
        );
    } else {
        panic!("Expected text message");
    }

    drop(broadcast_tx);
    tokio::time::timeout(Duration::from_millis(100), forward_handle)
        .await
        .expect("Task should exit cleanly")
        .unwrap();
}

#[tokio::test]
async fn test_async_forward_broadcast_handles_destination_close_cleanly() {
    let (broadcast_tx, broadcast_rx) = broadcast::channel(16);
    let (out_tx, out_rx) = mpsc::channel(16);

    let forward_handle = tokio::spawn(async move {
        forward_broadcast_to_browser(broadcast_rx, "/tf".to_string(), None, out_tx, 0).await;
    });

    drop(out_rx);

    let payload: Arc<str> = Arc::from("{\"transforms\":[]}");
    let _ = broadcast_tx.send(payload);

    tokio::time::timeout(Duration::from_millis(100), forward_handle)
        .await
        .expect("Task should exit when destination channel closes")
        .unwrap();
}

#[tokio::test]
async fn test_async_forward_broadcast_throttles_high_rate() {
    let (broadcast_tx, broadcast_rx) = broadcast::channel(16);
    let (out_tx, mut out_rx) = mpsc::channel(16);

    let forward_handle = tokio::spawn(async move {
        forward_broadcast_to_browser(
            broadcast_rx,
            "/tf".to_string(),
            None,
            out_tx,
            100, // 100ms throttle
        )
        .await;
    });

    broadcast_tx.send(Arc::from("{\"msg\":1}")).unwrap();
    broadcast_tx.send(Arc::from("{\"msg\":2}")).unwrap();
    broadcast_tx.send(Arc::from("{\"msg\":3}")).unwrap();

    let first = out_rx.recv().await.unwrap();
    if let Message::Text(text) = first {
        assert_eq!(text, "{\"op\":\"publish\",\"topic\":\"/tf\",\"msg\":{\"msg\":1}}");
    }

    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(out_rx.try_recv().is_err());

    tokio::time::sleep(Duration::from_millis(90)).await;
    broadcast_tx.send(Arc::from("{\"msg\":4}")).unwrap();

    let second = out_rx.recv().await.unwrap();
    if let Message::Text(text) = second {
        assert_eq!(text, "{\"op\":\"publish\",\"topic\":\"/tf\",\"msg\":{\"msg\":4}}");
    }

    drop(broadcast_tx);
    let _ = forward_handle.await;
}

#[test]
fn test_protocol_parsing_and_envelope_formatting() {
    let sub_json = r#"{"op":"subscribe","topic":"/tf","id":"tf_1","throttle_rate":20}"#;
    assert_eq!(
        parse_client_message(sub_json),
        ClientOp::Subscribe {
            topic: "/tf",
            id: Some("tf_1"),
            throttle_rate_ms: 20,
        }
    );

    let unsub_json = r#"{"op":"unsubscribe","topic":"/tf","id":"tf_1"}"#;
    assert_eq!(
        parse_client_message(unsub_json),
        ClientOp::Unsubscribe {
            topic: "/tf",
            id: Some("tf_1"),
        }
    );

    let formatted = format_rosbridge_publish("/tf", "{\"transforms\":[]}", Some("tf_1"));
    assert_eq!(
        formatted,
        "{\"op\":\"publish\",\"topic\":\"/tf\",\"id\":\"tf_1\",\"msg\":{\"transforms\":[]}}"
    );
}
