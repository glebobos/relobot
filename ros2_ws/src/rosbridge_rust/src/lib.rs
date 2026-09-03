//! # rosbridge_rust
//!
//! High-performance native ROS 2 rosbridge WebSocket server and Python proxy.
//!
//! Provides zero-copy JSON streaming for high-bandwidth topics (`/tf`, `/map`, `/robot_pose`)
//! while transparently routing service calls, actions, and secondary topics to a standard Python rosbridge backend.

#![deny(clippy::correctness)]
#![warn(clippy::suspicious, clippy::style, clippy::perf)]

pub mod client;
pub mod error;
pub mod protocol;
pub mod ros;
pub mod state;
pub mod upstream;
pub mod watchdog;

pub use client::{forward_broadcast_to_browser, handle_client_connection};
pub use error::{BridgeError, Result};
pub use protocol::{format_rosbridge_publish, parse_client_message, ClientOp};
pub use ros::{init_ros2_node, Ros2Tasks};
pub use state::{
    fresh_cached_message, monotonic_ms, CachedMessage, RateLimiter, SharedState, SubscriptionKey,
};
pub use upstream::{run_upstream_manager, SubscriptionRegistry};
pub use watchdog::run_watchdog;
