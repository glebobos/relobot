//! WebSocket client session management and message routing.

use crate::error::Result;
use crate::protocol::{format_rosbridge_publish, parse_client_message, ClientOp};
use crate::state::{fresh_cached_message, monotonic_ms, RateLimiter, SharedState, SubscriptionKey};
use crate::upstream::{run_upstream_manager, SubscriptionRegistry};
use futures_util::{SinkExt, StreamExt};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::TcpStream;
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;
use tokio_tungstenite::tungstenite::Message;
use tracing::{debug, instrument, warn};

/// Streams broadcast topic updates to an MPSC sink for a client subscription with pacing.
pub async fn forward_broadcast_to_browser(
    mut rx: broadcast::Receiver<Arc<str>>,
    topic: String,
    id: Option<String>,
    tx: mpsc::Sender<Message>,
    throttle_rate_ms: u64,
) {
    let mut limiter = RateLimiter::new(Duration::from_millis(throttle_rate_ms));
    loop {
        match rx.recv().await {
            Ok(json_payload) => {
                let now = Instant::now();
                if !limiter.should_process(now) {
                    continue;
                }
                let frame = format_rosbridge_publish(&topic, &json_payload, id.as_deref());
                if tx.send(Message::Text(frame)).await.is_err() {
                    break;
                }
            }
            Err(broadcast::error::RecvError::Lagged(n)) => {
                warn!(lagged_count = n, topic = %topic, "Client subscriber lagged on broadcast stream");
                continue;
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
}

/// Handles a single incoming browser WebSocket client connection.
#[instrument(skip_all, fields(client = %peer_addr))]
pub async fn handle_client_connection(
    stream: TcpStream,
    peer_addr: SocketAddr,
    state: Arc<SharedState>,
    upstream_url: &str,
) -> Result<()> {
    let ws_stream = tokio_tungstenite::accept_async(stream).await?;
    let (mut browser_tx, mut browser_rx) = ws_stream.split();

    // Outgoing channel for messages destined to the browser WebSocket
    let (browser_out_tx, mut browser_out_rx) = mpsc::channel::<Message>(256);

    // Channel for forwarding upstream commands to the secondary Python backend
    let (py_in_tx, py_in_rx) = mpsc::channel::<Message>(128);

    // Registry of active non-local subscriptions for upstream auto-recovery
    let active_python_subs = SubscriptionRegistry::new();

    // Monotonic timestamp tracking client activity
    let last_browser_activity = Arc::new(AtomicU64::new(monotonic_ms()));

    // 1. Task: Forward MPSC channel to Browser WebSocket with timeout protection
    let browser_writer = tokio::spawn(async move {
        while let Some(msg) = browser_out_rx.recv().await {
            match tokio::time::timeout(Duration::from_secs(5), browser_tx.send(msg)).await {
                Ok(Ok(())) => {}
                Ok(Err(e)) => {
                    debug!(error = %e, "Browser WebSocket write failed");
                    break;
                }
                Err(_) => {
                    warn!("Browser WebSocket write timed out (>5s); disconnecting slow client");
                    break;
                }
            }
        }
    });

    // 2. Task: Send WebSocket Ping keep-alive every 5s and prune ghost sockets
    let browser_out_tx_ping = browser_out_tx.clone();
    let activity_tracker = last_browser_activity.clone();
    let keepalive_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(5));
        loop {
            interval.tick().await;
            let now = monotonic_ms();
            let last_act = activity_tracker.load(Ordering::Relaxed);
            if now.saturating_sub(last_act) > 20_000 {
                warn!("Client keep-alive timeout (>20s silent); pruning dead connection");
                let _ = browser_out_tx_ping.send(Message::Close(None)).await;
                break;
            }
            if browser_out_tx_ping.send(Message::Ping(vec![])).await.is_err() {
                break;
            }
        }
    });

    // 3. Upstream Python Backend Manager Task
    let browser_out_tx_py = browser_out_tx.clone();
    let active_subs_clone = active_python_subs.clone();
    let upstream_url_owned = upstream_url.to_string();
    let python_manager = tokio::spawn(async move {
        run_upstream_manager(
            &upstream_url_owned,
            py_in_rx,
            browser_out_tx_py,
            active_subs_clone,
        )
        .await;
    });

    // 4. Map of active local topic subscription tasks
    let mut active_subs: HashMap<SubscriptionKey, JoinHandle<()>> = HashMap::new();

    // 5. Client WebSocket receive loop
    while let Some(msg_result) = browser_rx.next().await {
        last_browser_activity.store(monotonic_ms(), Ordering::Relaxed);

        let msg = match msg_result {
            Ok(m) => m,
            Err(e) => {
                debug!(error = %e, "Browser WebSocket read ended");
                break;
            }
        };

        if msg.is_pong() {
            continue;
        }

        if msg.is_text() {
            let text = msg.to_text()?;
            let client_op = parse_client_message(text);

            match client_op {
                ClientOp::Subscribe {
                    topic,
                    id,
                    throttle_rate_ms,
                } if is_local_topic(topic) => {
                    let sub_key = SubscriptionKey::new(topic, id.map(ToString::to_string));
                    if let std::collections::hash_map::Entry::Vacant(vacant) = active_subs.entry(sub_key) {
                        debug!(topic = %topic, "Subscribing locally to ROS 2 topic");

                        let browser_out_tx_clone = browser_out_tx.clone();
                        let state_clone = state.clone();
                        let topic_str = topic.to_string();
                        let id_clone = id.map(ToString::to_string);

                        let handle = tokio::spawn(async move {
                            let now = Instant::now();
                            // 1. Send cached message immediately if fresh
                            let cached = match topic_str.as_str() {
                                "/tf" => state_clone.tf_cache.read().await.clone(),
                                "/map" => state_clone.map_cache.read().await.clone(),
                                "/robot_pose" => state_clone.pose_cache.read().await.clone(),
                                _ => None,
                            };

                            if let Some(cached_payload) =
                                fresh_cached_message(&topic_str, cached, now)
                            {
                                let frame = format_rosbridge_publish(
                                    &topic_str,
                                    &cached_payload,
                                    id_clone.as_deref(),
                                );
                                let _ = browser_out_tx_clone.send(Message::Text(frame)).await;
                            }

                            // 2. Stream broadcast updates
                            match topic_str.as_str() {
                                "/tf" => {
                                    let rx = state_clone.tf_tx.subscribe();
                                    forward_broadcast_to_browser(
                                        rx,
                                        topic_str,
                                        id_clone,
                                        browser_out_tx_clone,
                                        throttle_rate_ms,
                                    )
                                    .await;
                                }
                                "/map" => {
                                    let rx = state_clone.map_tx.subscribe();
                                    forward_broadcast_to_browser(
                                        rx,
                                        topic_str,
                                        id_clone,
                                        browser_out_tx_clone,
                                        throttle_rate_ms,
                                    )
                                    .await;
                                }
                                "/robot_pose" => {
                                    let rx = state_clone.pose_tx.subscribe();
                                    forward_broadcast_to_browser(
                                        rx,
                                        topic_str,
                                        id_clone,
                                        browser_out_tx_clone,
                                        throttle_rate_ms,
                                    )
                                    .await;
                                }
                                _ => {}
                            }
                        });
                        vacant.insert(handle);
                    }
                }
                ClientOp::Unsubscribe { topic, id } if is_local_topic(topic) => {
                    let sub_key = SubscriptionKey::new(topic, id.map(ToString::to_string));
                    if let Some(handle) = active_subs.remove(&sub_key) {
                        debug!(topic = %topic, "Unsubscribing locally from ROS 2 topic");
                        handle.abort();
                    }
                }
                ClientOp::Subscribe { topic, id, .. } => {
                    let sub_key = SubscriptionKey::new(topic, id.map(ToString::to_string));
                    active_python_subs
                        .register(sub_key, text.to_string())
                        .await;
                    forward_to_upstream(&py_in_tx, msg).await;
                }
                ClientOp::Unsubscribe { topic, id } => {
                    let sub_key = SubscriptionKey::new(topic, id.map(ToString::to_string));
                    active_python_subs.unregister(&sub_key).await;
                    forward_to_upstream(&py_in_tx, msg).await;
                }
                ClientOp::Other => {
                    forward_to_upstream(&py_in_tx, msg).await;
                }
            }
        } else if msg.is_binary() {
            forward_to_upstream(&py_in_tx, msg).await;
        } else if msg.is_close() {
            break;
        }
    }

    // Cleanup all task handles for this connection
    for (_, handle) in active_subs {
        handle.abort();
    }
    browser_writer.abort();
    keepalive_task.abort();
    python_manager.abort();

    Ok(())
}

#[inline]
fn is_local_topic(topic: &str) -> bool {
    matches!(topic, "/tf" | "/map" | "/robot_pose")
}

async fn forward_to_upstream(py_in_tx: &mpsc::Sender<Message>, msg: Message) {
    if let Err(e) = py_in_tx.try_send(msg) {
        match e {
            mpsc::error::TrySendError::Full(msg_back) => {
                debug!("Upstream command queue full; awaiting capacity...");
                let _ = py_in_tx.send(msg_back).await;
            }
            mpsc::error::TrySendError::Closed(_) => {
                warn!("Upstream command channel closed while routing message");
            }
        }
    }
}
