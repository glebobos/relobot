//! Native ROS 2 node setup, topic subscriptions, and background spin management.

use crate::error::Result;
use crate::state::{monotonic_ms, CachedMessage, RateLimiter, SharedState};
use futures_util::StreamExt;
use r2r::QosProfile;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use tracing::{info, warn};

/// Handles to all spawned background ROS 2 tasks.
pub struct Ros2Tasks {
    /// Dedicated OS thread handle running `node.spin_once`.
    pub spin_thread: JoinHandle<()>,
    /// Tokio task handle for `/tf` stream processing.
    pub tf_task: tokio::task::JoinHandle<()>,
    /// Tokio task handle for `/map` stream processing.
    pub map_task: tokio::task::JoinHandle<()>,
    /// Tokio task handle for `/robot_pose` stream processing.
    pub pose_task: tokio::task::JoinHandle<()>,
}

/// Initializes the ROS 2 node, attaches native subscriptions, and spawns background tasks.
pub fn init_ros2_node(state: Arc<SharedState>) -> Result<Ros2Tasks> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rosbridge_rust", "")?;

    // 1. Subscribe to /tf
    let mut tf_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>("/tf", QosProfile::default())?;

    // 2. Subscribe to /map with Transient Local QoS
    let mut map_sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(
        "/map",
        QosProfile::default()
            .transient_local()
            .reliable()
            .keep_last(1),
    )?;

    // 3. Subscribe to /robot_pose
    let mut pose_sub = node.subscribe::<r2r::geometry_msgs::msg::PoseStamped>(
        "/robot_pose",
        QosProfile::default(),
    )?;

    // Spawn dedicated OS spin thread
    let state_spin = state.clone();
    let spin_thread = std::thread::Builder::new()
        .name("ros2_spin".to_string())
        .spawn(move || {
            info!("ROS 2 spin thread started");
            loop {
                node.spin_once(Duration::from_millis(10));
                state_spin.update_spin_heartbeat();
                std::thread::sleep(Duration::from_millis(10));
            }
        })?;

    // Spawn TF subscriber task (throttled to ~20 Hz)
    let state_tf = state.clone();
    let tf_task = tokio::spawn(async move {
        let mut limiter = RateLimiter::new(Duration::from_millis(50));
        info!("TF subscriber task started (max 20 Hz)");
        while let Some(msg) = tf_sub.next().await {
            let now = Instant::now();
            if limiter.should_process(now) {
                state_tf.last_tf_at_ms.store(monotonic_ms(), Ordering::Relaxed);
                if let Ok(json_str) = serde_json::to_string(&msg) {
                    let payload: Arc<str> = Arc::from(json_str);
                    *state_tf.tf_cache.write().await = Some(CachedMessage::new(payload.clone()));
                    let _ = state_tf.tf_tx.send(payload);
                }
            }
        }
        warn!("TF subscriber stream ended");
    });

    // Spawn Map subscriber task (strict max 1.0 Hz with offloaded JSON serialization)
    let state_map = state.clone();
    let map_task = tokio::spawn(async move {
        let mut limiter = RateLimiter::new(Duration::from_millis(1000));
        info!("Map subscriber task started (max 1.0 Hz, offloaded serialization)");
        while let Some(msg) = map_sub.next().await {
            let now = Instant::now();
            if limiter.should_process(now) {
                state_map.last_map_at_ms.store(monotonic_ms(), Ordering::Relaxed);
                // Offload heavy OccupancyGrid serialization to a worker thread pool
                let serialize_result =
                    tokio::task::spawn_blocking(move || serde_json::to_string(&msg)).await;

                if let Ok(Ok(json_str)) = serialize_result {
                    let payload: Arc<str> = Arc::from(json_str);
                    *state_map.map_cache.write().await =
                        Some(CachedMessage::new(payload.clone()));
                    let _ = state_map.map_tx.send(payload);
                }
            }
        }
        warn!("Map subscriber stream ended");
    });

    // Spawn Robot Pose subscriber task (throttled to ~20 Hz)
    let state_pose = state.clone();
    let pose_task = tokio::spawn(async move {
        let mut limiter = RateLimiter::new(Duration::from_millis(50));
        info!("Robot pose subscriber task started (max 20 Hz)");
        while let Some(msg) = pose_sub.next().await {
            let now = Instant::now();
            if limiter.should_process(now) {
                state_pose.last_pose_at_ms.store(monotonic_ms(), Ordering::Relaxed);
                if let Ok(json_str) = serde_json::to_string(&msg) {
                    let payload: Arc<str> = Arc::from(json_str);
                    *state_pose.pose_cache.write().await =
                        Some(CachedMessage::new(payload.clone()));
                    let _ = state_pose.pose_tx.send(payload);
                }
            }
        }
        warn!("Robot pose subscriber stream ended");
    });

    Ok(Ros2Tasks {
        spin_thread,
        tf_task,
        map_task,
        pose_task,
    })
}
