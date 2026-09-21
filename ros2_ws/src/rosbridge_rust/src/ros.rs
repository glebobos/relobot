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

    // Spawn Map subscriber task (strict max 1.0 Hz with offloaded binary encoding & JSON serialization)
    let state_map = state.clone();
    let map_task = tokio::spawn(async move {
        let mut limiter = RateLimiter::new(Duration::from_millis(1000));
        info!("Map subscriber task started (max 1.0 Hz, binary compression & JSON serialization)");
        while let Some(msg) = map_sub.next().await {
            let now = Instant::now();
            if limiter.should_process(now) {
                state_map.last_map_at_ms.store(monotonic_ms(), Ordering::Relaxed);

                // Offload heavy binary encoding & JSON serialization to worker thread pool
                let process_result = tokio::task::spawn_blocking(move || {
                    let binary_frame = encode_binary_occupancy_grid(&msg);
                    let json_str = serde_json::to_string(&msg);
                    (binary_frame, json_str)
                })
                .await;

                if let Ok((binary_frame, json_result)) = process_result {
                    // Update binary map cache & broadcast
                    let binary_payload: Arc<[u8]> = Arc::from(binary_frame);
                    *state_map.binary_map_cache.write().await = Some(binary_payload.clone());
                    let _ = state_map.binary_map_tx.send(binary_payload);

                    // Update standard JSON map cache & broadcast for backwards compatibility
                    if let Ok(json_str) = json_result {
                        let payload: Arc<str> = Arc::from(json_str);
                        *state_map.map_cache.write().await =
                            Some(CachedMessage::new(payload.clone()));
                        let _ = state_map.map_tx.send(payload);
                    }
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

pub const BINARY_MAP_MAGIC: &[u8; 4] = b"RMAP";
pub const BINARY_MAP_VERSION: u8 = 1;
pub const COMPRESSION_ZLIB: u8 = 1;
pub const BINARY_MAP_HEADER_SIZE: usize = 48;

/// Encodes an OccupancyGrid into a high-performance compressed binary frame.
///
/// Header (48 bytes, little-endian):
/// - [0..4]   magic: b"RMAP"
/// - [4]      version: 1
/// - [5]      compression: 1 (ZLIB/Deflate)
/// - [6..8]   reserved: [0, 0]
/// - [8..12]  width: u32
/// - [12..16] height: u32
/// - [16..20] resolution: f32
/// - [20..24] origin_x: f32
/// - [24..28] origin_y: f32
/// - [28..32] origin_z: f32
/// - [32..36] origin_qx: f32
/// - [36..40] origin_qy: f32
/// - [40..44] origin_qz: f32
/// - [44..48] origin_qw: f32
///
/// Followed by zlib-compressed raw [i8] data.
pub fn encode_binary_occupancy_grid(msg: &r2r::nav_msgs::msg::OccupancyGrid) -> Vec<u8> {
    let mut buf = Vec::with_capacity(BINARY_MAP_HEADER_SIZE + (msg.data.len() / 4).max(1024));

    // Magic & Protocol Metadata
    buf.extend_from_slice(BINARY_MAP_MAGIC);
    buf.push(BINARY_MAP_VERSION);
    buf.push(COMPRESSION_ZLIB);
    buf.extend_from_slice(&[0u8, 0u8]); // reserved

    // Map Dimensions & Spatial Parameters (little-endian)
    buf.extend_from_slice(&msg.info.width.to_le_bytes());
    buf.extend_from_slice(&msg.info.height.to_le_bytes());
    buf.extend_from_slice(&msg.info.resolution.to_le_bytes());

    let origin = &msg.info.origin;
    buf.extend_from_slice(&(origin.position.x as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.position.y as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.position.z as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.orientation.x as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.orientation.y as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.orientation.z as f32).to_le_bytes());
    buf.extend_from_slice(&(origin.orientation.w as f32).to_le_bytes());

    debug_assert_eq!(buf.len(), BINARY_MAP_HEADER_SIZE);

    // Cast [i8] to [u8] slice safely
    // SAFETY: i8 and u8 have identical memory layout, size (1 byte), alignment (1 byte),
    // and validity (all 256 bit patterns are valid for both u8 and i8).
    let raw_bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(msg.data.as_ptr() as *const u8, msg.data.len())
    };

    // Fast zlib compression (level 1: fastest, ~0.8ms for 800x800, 95%+ compression on sparse grids)
    let compressed = miniz_oxide::deflate::compress_to_vec_zlib(raw_bytes, 1);
    buf.extend_from_slice(&compressed);

    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_binary_map_roundtrip_fidelity() {
        let mut msg = r2r::nav_msgs::msg::OccupancyGrid::default();
        msg.info.width = 400;
        msg.info.height = 300;
        msg.info.resolution = 0.05;
        msg.info.origin.position.x = -10.5;
        msg.info.origin.position.y = -7.25;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        let total_cells = (msg.info.width * msg.info.height) as usize;
        let mut test_data = vec![-1i8; total_cells];
        // Populate varied values
        for i in 0..1000 {
            test_data[i * 2] = 0;
            test_data[i * 3] = 100;
        }
        msg.data = test_data.clone();

        let encoded = encode_binary_occupancy_grid(&msg);

        // Header checks
        assert!(encoded.len() > BINARY_MAP_HEADER_SIZE);
        assert_eq!(&encoded[0..4], BINARY_MAP_MAGIC);
        assert_eq!(encoded[4], BINARY_MAP_VERSION);
        assert_eq!(encoded[5], COMPRESSION_ZLIB);

        let width = u32::from_le_bytes(encoded[8..12].try_into().unwrap());
        let height = u32::from_le_bytes(encoded[12..16].try_into().unwrap());
        let resolution = f32::from_le_bytes(encoded[16..20].try_into().unwrap());
        assert_eq!(width, 400);
        assert_eq!(height, 300);
        assert!((resolution - 0.05).abs() < 1e-6);

        let ox = f32::from_le_bytes(encoded[20..24].try_into().unwrap());
        let oy = f32::from_le_bytes(encoded[24..28].try_into().unwrap());
        assert!((ox - (-10.5)).abs() < 1e-5);
        assert!((oy - (-7.25)).abs() < 1e-5);

        // Decompress payload and verify 100% data equality (no downsampling)
        let decompressed = miniz_oxide::inflate::decompress_to_vec_zlib(&encoded[BINARY_MAP_HEADER_SIZE..])
            .expect("Decompression failed");
        assert_eq!(decompressed.len(), total_cells);

        let decoded_data: &[i8] = unsafe {
            std::slice::from_raw_parts(decompressed.as_ptr() as *const i8, decompressed.len())
        };
        assert_eq!(decoded_data, test_data.as_slice());
    }

    #[test]
    fn test_binary_map_compression_ratio() {
        let mut msg = r2r::nav_msgs::msg::OccupancyGrid::default();
        msg.info.width = 800;
        msg.info.height = 800;
        msg.info.resolution = 0.05;

        // Typical sparse indoor/garden grid (mostly unknown and free space)
        let total = 800 * 800;
        let mut data = vec![-1i8; total];
        for i in 200..600 {
            for j in 200..600 {
                data[i * 800 + j] = 0;
            }
        }
        // Border walls
        for i in 200..600 {
            data[i * 800 + 200] = 100;
            data[i * 800 + 599] = 100;
        }
        msg.data = data;

        let encoded = encode_binary_occupancy_grid(&msg);
        let raw_size = total; // 640,000 bytes
        let compressed_size = encoded.len();

        // Expect massive compression (over 90% reduction, under 60KB)
        assert!(
            compressed_size < 60_000,
            "Compressed size was {} bytes (raw was {} bytes)",
            compressed_size,
            raw_size
        );
    }
}

