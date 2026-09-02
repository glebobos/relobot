//! Upstream proxy manager connecting to the secondary Python rosbridge backend.

use crate::state::SubscriptionKey;
use futures_util::{SinkExt, StreamExt};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{mpsc, RwLock};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use tracing::{debug, info, warn};

/// Thread-safe registry storing active non-local subscriptions to replay upon reconnection.
#[derive(Clone, Default)]
pub struct SubscriptionRegistry {
    entries: Arc<RwLock<HashMap<SubscriptionKey, String>>>,
}

impl SubscriptionRegistry {
    /// Creates an empty subscription registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers a subscription with its original JSON message.
    pub async fn register(&self, key: SubscriptionKey, raw_msg: String) {
        self.entries.write().await.insert(key, raw_msg);
    }

    /// Removes a subscription by key.
    pub async fn unregister(&self, key: &SubscriptionKey) {
        self.entries.write().await.remove(key);
    }

    /// Returns a snapshot of all active subscription payloads.
    pub async fn snapshot(&self) -> Vec<String> {
        self.entries.write().await.values().cloned().collect()
    }
}

/// Runs the upstream Python rosbridge connection manager and bi-directional message proxy.
///
/// Automatically reconnects with exponential backoff on disconnect and replays active subscriptions.
pub async fn run_upstream_manager(
    upstream_url: &str,
    mut py_in_rx: mpsc::Receiver<Message>,
    browser_out_tx: mpsc::Sender<Message>,
    registry: SubscriptionRegistry,
) {
    let mut backoff_ms = 200;

    loop {
        match connect_async(upstream_url).await {
            Ok((py_ws, _)) => {
                info!(url = %upstream_url, "Connected to Python rosbridge backend");
                backoff_ms = 200;

                let (mut py_tx, mut py_rx) = py_ws.split();

                // Replay all active subscriptions registered by the client
                let subscriptions = registry.snapshot().await;
                for sub_msg in subscriptions {
                    debug!(sub = %sub_msg, "Replaying subscription to upstream Python backend");
                    if let Err(e) = py_tx.send(Message::Text(sub_msg)).await {
                        warn!(error = %e, "Failed to replay subscription to upstream");
                        break;
                    }
                }

                // Bi-directional message routing between Python WS and client MPSC
                loop {
                    tokio::select! {
                        py_msg = py_rx.next() => {
                            match py_msg {
                                Some(Ok(msg)) => {
                                    if (msg.is_text() || msg.is_binary())
                                        && browser_out_tx.send(msg).await.is_err()
                                    {
                                        debug!("Client output channel closed; terminating upstream manager");
                                        return;
                                    }
                                }
                                Some(Err(e)) => {
                                    warn!(error = %e, "Python backend socket error; reconnecting...");
                                    break;
                                }
                                None => {
                                    warn!("Python backend closed connection; reconnecting...");
                                    break;
                                }
                            }
                        }
                        browser_cmd = py_in_rx.recv() => {
                            match browser_cmd {
                                Some(msg) => {
                                    if let Err(e) = py_tx.send(msg).await {
                                        warn!(error = %e, "Failed to send command to Python; triggering reconnect...");
                                        break;
                                    }
                                }
                                None => {
                                    debug!("Browser command channel closed; terminating upstream manager");
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            Err(e) => {
                tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
                backoff_ms = std::cmp::min(backoff_ms * 2, 2000);

                if browser_out_tx.is_closed() {
                    return;
                }
                debug!(
                    error = %e,
                    retry_in_ms = backoff_ms,
                    "Waiting for Python backend; retrying..."
                );
            }
        }
    }
}
