use futures_util::{SinkExt, StreamExt};
use r2r::QosProfile;
use serde_json::Value;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, mpsc, RwLock};
use tokio::task::JoinHandle;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

struct SharedState {
    tf_json: RwLock<Option<Value>>,
    map_json: RwLock<Option<Value>>,
    pose_json: RwLock<Option<Value>>,
    tf_tx: broadcast::Sender<Value>,
    map_tx: broadcast::Sender<Value>,
    pose_tx: broadcast::Sender<Value>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("[RosbridgeRust] Starting up...");

    // Initialize ROS 2 Node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rosbridge_rust", "")?;

    // Broadcast channels for active topics
    let (tf_tx, _) = broadcast::channel(100);
    let (map_tx, _) = broadcast::channel(10);
    let (pose_tx, _) = broadcast::channel(100);

    let state = Arc::new(SharedState {
        tf_json: RwLock::new(None),
        map_json: RwLock::new(None),
        pose_json: RwLock::new(None),
        tf_tx: tf_tx.clone(),
        map_tx: map_tx.clone(),
        pose_tx: pose_tx.clone(),
    });

    // Native ROS 2 Subscriptions
    let mut tf_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>("/tf", QosProfile::default())?;
    let mut map_sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(
        "/map",
        QosProfile::default()
            .transient_local()
            .reliable()
            .keep_last(1),
    )?;
    let mut pose_sub = node.subscribe::<r2r::geometry_msgs::msg::PoseStamped>(
        "/robot_pose",
        QosProfile::default(),
    )?;

    // Spawn ROS 2 background spin task in a dedicated OS thread
    let _spin_thread = std::thread::Builder::new()
        .name("ros2_spin".to_string())
        .spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                println!("[RosbridgeRust] ROS2 spin thread started.");
                loop {
                    node.spin_once(std::time::Duration::from_millis(10));
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
            }));
            eprintln!("[RosbridgeRust] FATAL: ROS2 spin thread panicked or exited: {:?}", result);
            std::process::exit(1);
        })?;

    // Spawn tf subscriber task
    let state_clone_tf = state.clone();
    let tf_handle = tokio::spawn(async move {
        let mut last_sent: Option<std::time::Instant> = None;
        let throttle_duration = std::time::Duration::from_millis(40); // Max ~25 Hz
        println!("[RosbridgeRust] TF subscriber task started");
        while let Some(msg) = tf_sub.next().await {
            let now = std::time::Instant::now();
            let should_send = match last_sent {
                None => true,
                Some(last) => now.duration_since(last) >= throttle_duration,
            };
            if should_send {
                if let Ok(val) = serde_json::to_value(&msg) {
                    *state_clone_tf.tf_json.write().await = Some(val.clone());
                    let _ = state_clone_tf.tf_tx.send(val);
                }
                last_sent = Some(now);
            }
        }
        println!("[RosbridgeRust] WARNING: TF subscriber stream ended");
    });

    // Spawn map subscriber task
    let state_clone_map = state.clone();
    let map_handle = tokio::spawn(async move {
        println!("[RosbridgeRust] Map subscriber task started");
        while let Some(msg) = map_sub.next().await {
            if let Ok(val) = serde_json::to_value(&msg) {
                *state_clone_map.map_json.write().await = Some(val.clone());
                let _ = state_clone_map.map_tx.send(val);
            }
        }
        println!("[RosbridgeRust] WARNING: Map subscriber stream ended");
    });

    // Spawn robot pose subscriber task
    let state_clone_pose = state.clone();
    let pose_handle = tokio::spawn(async move {
        println!("[RosbridgeRust] Robot pose subscriber task started");
        while let Some(msg) = pose_sub.next().await {
            if let Ok(val) = serde_json::to_value(&msg) {
                *state_clone_pose.pose_json.write().await = Some(val.clone());
                let _ = state_clone_pose.pose_tx.send(val);
            }
        }
        println!("[RosbridgeRust] WARNING: Robot pose subscriber stream ended");
    });

    // Spawn task to monitor subscriber health
    tokio::spawn(async move {
        tokio::select! {
            res = tf_handle => {
                eprintln!("[RosbridgeRust] FATAL: TF subscriber task exited: {:?}", res);
            }
            res = map_handle => {
                eprintln!("[RosbridgeRust] FATAL: Map subscriber task exited: {:?}", res);
            }
            res = pose_handle => {
                eprintln!("[RosbridgeRust] FATAL: Robot pose subscriber task exited: {:?}", res);
            }
        }
        std::process::exit(1);
    });

    // Start WebSocket Server on 0.0.0.0:9090
    let addr: SocketAddr = "0.0.0.0:9090".parse()?;
    let listener = TcpListener::bind(&addr).await?;
    println!("[RosbridgeRust] Listening on {}", addr);

    while let Ok((stream, _)) = listener.accept().await {
        let state_clone = state.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(stream, state_clone).await {
                println!("[RosbridgeRust] Error handling connection: {:?}", e);
            }
        });
    }

    Ok(())
}

async fn handle_connection(
    stream: TcpStream,
    state: Arc<SharedState>,
) -> Result<(), Box<dyn std::error::Error>> {
    let ws_stream = tokio_tungstenite::accept_async(stream).await?;
    let (mut browser_tx, mut browser_rx) = ws_stream.split();

    println!("[RosbridgeRust] Browser client connected.");

    // Connect to Python rosbridge on port 9091
    let mut retry_count = 0;
    let py_ws = loop {
        match connect_async("ws://127.0.0.1:9091").await {
            Ok((ws, _)) => break ws,
            Err(e) => {
                if retry_count > 5 {
                    return Err(Box::new(e));
                }
                retry_count += 1;
                tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
            }
        }
    };
    let (mut py_tx, mut py_rx) = py_ws.split();
    println!("[RosbridgeRust] Connected to Python backend on port 9091.");

    // Thread-safe MPSC channel for browser output so both tasks can send messages to the browser
    let (browser_out_tx, mut browser_out_rx) = mpsc::channel::<Message>(100);
    
    // Task to forward MPSC channel to Browser WebSocket
    let browser_writer = tokio::spawn(async move {
        while let Some(msg) = browser_out_rx.recv().await {
            if browser_tx.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Task to forward Python backend WebSocket messages back to browser
    let browser_out_tx_clone = browser_out_tx.clone();
    let python_forwarder = tokio::spawn(async move {
        while let Some(Ok(msg)) = py_rx.next().await {
            if browser_out_tx_clone.send(msg).await.is_err() {
                break;
            }
        }
    });

    // Keep track of local subscription tasks
    let mut active_subs: HashMap<(String, Option<String>), JoinHandle<()>> = HashMap::new();

    // Loop reading messages from browser WebSocket
    while let Some(Ok(msg)) = browser_rx.next().await {
        if msg.is_text() {
            let text = msg.to_text()?;
            if let Ok(json_val) = serde_json::from_str::<Value>(text) {
                let op = json_val["op"].as_str().unwrap_or("");
                let topic = json_val["topic"].as_str().unwrap_or("");
                let id = json_val["id"].as_str().map(|s| s.to_string());

                if op == "subscribe" && (topic == "/tf" || topic == "/map" || topic == "/robot_pose") {
                    let key = (topic.to_string(), id.clone());
                    if !active_subs.contains_key(&key) {
                        println!("[RosbridgeRust] Subscribing locally to {}", topic);
                        
                        // Spawn task to stream this topic to the browser
                        let browser_out_tx_clone = browser_out_tx.clone();
                        let state_clone = state.clone();
                        let topic_str = topic.to_string();
                        let id_clone = id.clone();

                        let handle = tokio::spawn(async move {
                            // 1. Send cached value first (if available)
                            let cached = match topic_str.as_str() {
                                "/tf" => state_clone.tf_json.read().await.clone(),
                                "/map" => state_clone.map_json.read().await.clone(),
                                "/robot_pose" => state_clone.pose_json.read().await.clone(),
                                _ => None,
                            };

                            if let Some(cached_val) = cached {
                                let mut response = serde_json::json!({
                                    "op": "publish",
                                    "topic": topic_str,
                                    "msg": cached_val
                                });
                                if let Some(ref sub_id) = id_clone {
                                    response["id"] = Value::String(sub_id.clone());
                                }
                                if let Ok(resp_str) = serde_json::to_string(&response) {
                                    let _ = browser_out_tx_clone.send(Message::Text(resp_str)).await;
                                }
                            }

                            // 2. Subscribe to broadcast channel
                            match topic_str.as_str() {
                                "/tf" => {
                                    let rx = state_clone.tf_tx.subscribe();
                                    forward_broadcast_to_browser(rx, topic_str, id_clone, browser_out_tx_clone).await;
                                }
                                "/map" => {
                                    let rx = state_clone.map_tx.subscribe();
                                    forward_broadcast_to_browser(rx, topic_str, id_clone, browser_out_tx_clone).await;
                                }
                                "/robot_pose" => {
                                    let rx = state_clone.pose_tx.subscribe();
                                    forward_broadcast_to_browser(rx, topic_str, id_clone, browser_out_tx_clone).await;
                                }
                                _ => {}
                            }
                        });
                        active_subs.insert(key, handle);
                    }
                } else if op == "unsubscribe" && (topic == "/tf" || topic == "/map" || topic == "/robot_pose") {
                    let key = (topic.to_string(), id.clone());
                    if let Some(handle) = active_subs.remove(&key) {
                        println!("[RosbridgeRust] Unsubscribing locally from {}", topic);
                        handle.abort();
                    }
                } else {
                    // Forward all other messages to Python server
                    let _ = py_tx.send(msg).await;
                }
            } else {
                // Forward unparseable JSON to Python server
                let _ = py_tx.send(msg).await;
            }
        } else {
            // Forward binary messages to Python server
            let _ = py_tx.send(msg).await;
        }
    }

    // Clean up subscription tasks
    for (_, handle) in active_subs {
        handle.abort();
    }
    browser_writer.abort();
    python_forwarder.abort();

    println!("[RosbridgeRust] Client disconnected.");
    Ok(())
}

async fn forward_broadcast_to_browser(
    mut rx: broadcast::Receiver<Value>,
    topic: String,
    id: Option<String>,
    tx: mpsc::Sender<Message>,
) {
    loop {
        match rx.recv().await {
            Ok(val) => {
                let mut response = serde_json::json!({
                    "op": "publish",
                    "topic": topic,
                    "msg": val
                });
                if let Some(ref sub_id) = id {
                    response["id"] = Value::String(sub_id.clone());
                }
                if let Ok(resp_str) = serde_json::to_string(&response) {
                    if tx.send(Message::Text(resp_str)).await.is_err() {
                        break;
                    }
                }
            }
            Err(broadcast::error::RecvError::Lagged(n)) => {
                eprintln!("[RosbridgeRust] Lagged {} messages on {}", n, topic);
                continue;
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
}
