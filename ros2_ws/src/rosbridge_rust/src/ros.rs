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
                // Offload heavy OccupancyGrid downsampling & serialization to worker thread pool
                let serialize_result = tokio::task::spawn_blocking(move || {
                    let optimized_msg = downsample_occupancy_grid(msg);
                    serde_json::to_string(&optimized_msg)
                })
                .await;

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

/// Adaptively downsamples large OccupancyGrid messages using max-pooling to preserve obstacles
/// while reducing network payload by 75% for web browsers.
#[inline]
pub fn downsample_occupancy_grid(
    mut msg: r2r::nav_msgs::msg::OccupancyGrid,
) -> r2r::nav_msgs::msg::OccupancyGrid {
    let w = msg.info.width as usize;
    let h = msg.info.height as usize;
    if w <= 400 && h <= 400 {
        return msg;
    }
    if w == 0 || h == 0 || msg.data.len() != w * h {
        return msg;
    }

    let scale = 2usize;
    let new_w = (w + scale - 1) / scale;
    let new_h = (h + scale - 1) / scale;
    let mut new_data = vec![-1i8; new_w * new_h];

    for r in 0..h {
        let new_r = r / scale;
        let r_offset = r * w;
        let new_r_offset = new_r * new_w;
        for c in 0..w {
            let new_c = c / scale;
            let val = msg.data[r_offset + c];
            let target = &mut new_data[new_r_offset + new_c];
            // Max-pooling:
            // 100 (obstacle) takes highest priority.
            // 0 (free) takes priority over -1 (unknown).
            if *target == 100 {
                continue;
            }
            if val == 100 {
                *target = 100;
            } else if val == 0 && *target == -1 {
                *target = 0;
            }
        }
    }

    msg.info.width = new_w as u32;
    msg.info.height = new_h as u32;
    msg.info.resolution *= scale as f32;
    msg.data = new_data;
    msg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_downsample_small_map_unchanged() {
        let mut msg = r2r::nav_msgs::msg::OccupancyGrid::default();
        msg.info.width = 100;
        msg.info.height = 100;
        msg.info.resolution = 0.05;
        msg.data = vec![0; 10000];

        let result = downsample_occupancy_grid(msg.clone());
        assert_eq!(result.info.width, 100);
        assert_eq!(result.info.height, 100);
        assert_eq!(result.info.resolution, 0.05);
        assert_eq!(result.data.len(), 10000);
    }

    #[test]
    fn test_downsample_large_map_with_obstacle_preservation() {
        let mut msg = r2r::nav_msgs::msg::OccupancyGrid::default();
        let w = 402;
        let h = 402;
        msg.info.width = w;
        msg.info.height = h;
        msg.info.resolution = 0.05;
        msg.data = vec![-1; (w * h) as usize];

        // Place obstacle at (0, 0) and free space at (2, 2)
        msg.data[0] = 100;
        msg.data[(2 * w + 2) as usize] = 0;

        let result = downsample_occupancy_grid(msg);
        assert_eq!(result.info.width, 201);
        assert_eq!(result.info.height, 201);
        assert_eq!(result.info.resolution, 0.10);
        assert_eq!(result.data.len(), (201 * 201) as usize);

        // Cell (0, 0) must preserve obstacle (100)
        assert_eq!(result.data[0], 100);
        // Cell (1, 1) corresponds to input (2, 2) and must be free space (0)
        assert_eq!(result.data[1 * 201 + 1], 0);
        // Untouched cell must remain unknown (-1)
        assert_eq!(result.data[200 * 201 + 200], -1);
    }
}

