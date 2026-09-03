//! Error types for the rosbridge proxy and native bridge server.

use thiserror::Error;

/// Result alias for rosbridge operations.
pub type Result<T, E = BridgeError> = std::result::Result<T, E>;

/// Domain-specific errors encountered across the rosbridge server.
#[derive(Debug, Error)]
pub enum BridgeError {
    /// Errors originating from the ROS 2 middleware interface (r2r).
    #[error("ros2 middleware error: {0}")]
    Ros(#[from] r2r::Error),

    /// Errors originating from WebSocket network operations (boxed to minimize enum size).
    #[error("websocket transport error: {0}")]
    WebSocket(Box<tokio_tungstenite::tungstenite::Error>),

    /// Errors in JSON serialization or deserialization.
    #[error("json serialization error: {0}")]
    Json(#[from] serde_json::Error),

    /// Standard I/O or network socket errors.
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    /// An internal async channel has closed unexpectedly.
    #[error("internal channel closed: {0}")]
    ChannelClosed(String),

    /// A connected client or upstream host timed out.
    #[error("operation timed out: {0}")]
    Timeout(String),
}

impl From<tokio_tungstenite::tungstenite::Error> for BridgeError {
    fn from(err: tokio_tungstenite::tungstenite::Error) -> Self {
        Self::WebSocket(Box::new(err))
    }
}
