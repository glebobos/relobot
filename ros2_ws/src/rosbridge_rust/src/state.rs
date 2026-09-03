//! Shared application state, topic caching, and concurrency utilities.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, RwLock};

static START_INSTANT: OnceLock<Instant> = OnceLock::new();

/// Returns strictly monotonic milliseconds elapsed since process start.
///
/// This is immune to wall-clock jumps, NTP updates, or pre-1970 embedded time resets.
#[must_use]
pub fn monotonic_ms() -> u64 {
    let start = START_INSTANT.get_or_init(Instant::now);
    start.elapsed().as_millis() as u64
}

/// A cached topic payload timestamped at arrival.
#[derive(Clone, Debug)]
pub struct CachedMessage {
    /// Pre-serialized JSON payload stored in a reference-counted slice.
    pub json_payload: Arc<str>,
    /// Instant when the message was received.
    pub received_at: Instant,
}

impl CachedMessage {
    /// Creates a new cached message stamped with current time.
    #[must_use]
    pub fn new(json_payload: Arc<str>) -> Self {
        Self {
            json_payload,
            received_at: Instant::now(),
        }
    }
}

/// Unique identifier for a topic subscription (topic name + optional client ID).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct SubscriptionKey {
    /// ROS 2 topic name (e.g., `"/tf"`).
    pub topic: String,
    /// Optional subscription identifier assigned by client.
    pub id: Option<String>,
}

impl SubscriptionKey {
    /// Creates a new subscription key.
    pub fn new(topic: impl Into<String>, id: Option<String>) -> Self {
        Self {
            topic: topic.into(),
            id,
        }
    }
}

/// Time-based rate limiter enforcing a minimum interval between emissions.
#[derive(Debug, Clone)]
pub struct RateLimiter {
    /// Minimum duration between allowed events.
    pub min_interval: Duration,
    /// Timestamp of the last allowed event.
    pub last_sent: Option<Instant>,
}

impl RateLimiter {
    /// Creates a new rate limiter with the specified minimum interval.
    #[must_use]
    pub const fn new(min_interval: Duration) -> Self {
        Self {
            min_interval,
            last_sent: None,
        }
    }

    /// Evaluates if an event should be processed at the given timestamp.
    ///
    /// Updates the internal timestamp if allowed.
    pub fn should_process(&mut self, now: Instant) -> bool {
        match self.last_sent {
            None => {
                self.last_sent = Some(now);
                true
            }
            Some(last) if now.duration_since(last) >= self.min_interval => {
                self.last_sent = Some(now);
                true
            }
            _ => false,
        }
    }
}

/// Shared application state across ROS 2 subscribers, client sessions, and watchdog.
pub struct SharedState {
    /// Most recent TF message cache.
    pub tf_cache: RwLock<Option<CachedMessage>>,
    /// Most recent OccupancyGrid map cache.
    pub map_cache: RwLock<Option<CachedMessage>>,
    /// Most recent RobotPose stamped cache.
    pub pose_cache: RwLock<Option<CachedMessage>>,

    /// Multi-producer broadcast channel for `/tf`.
    pub tf_tx: broadcast::Sender<Arc<str>>,
    /// Multi-producer broadcast channel for `/map`.
    pub map_tx: broadcast::Sender<Arc<str>>,
    /// Multi-producer broadcast channel for `/robot_pose`.
    pub pose_tx: broadcast::Sender<Arc<str>>,

    /// Monotonic timestamp of the last ROS 2 spin loop execution.
    pub spin_heartbeat_ms: AtomicU64,
    /// Total count of actively connected browser WebSocket clients.
    pub active_clients: AtomicU64,

    /// Monotonic timestamp of the last received TF message.
    pub last_tf_at_ms: AtomicU64,
    /// Monotonic timestamp of the last received Map message.
    pub last_map_at_ms: AtomicU64,
    /// Monotonic timestamp of the last received Pose message.
    pub last_pose_at_ms: AtomicU64,
}

impl SharedState {
    /// Initializes shared state with provided broadcast senders.
    #[must_use]
    pub fn new(
        tf_tx: broadcast::Sender<Arc<str>>,
        map_tx: broadcast::Sender<Arc<str>>,
        pose_tx: broadcast::Sender<Arc<str>>,
    ) -> Arc<Self> {
        Arc::new(Self {
            tf_cache: RwLock::new(None),
            map_cache: RwLock::new(None),
            pose_cache: RwLock::new(None),
            tf_tx,
            map_tx,
            pose_tx,
            spin_heartbeat_ms: AtomicU64::new(monotonic_ms()),
            active_clients: AtomicU64::new(0),
            last_tf_at_ms: AtomicU64::new(0),
            last_map_at_ms: AtomicU64::new(0),
            last_pose_at_ms: AtomicU64::new(0),
        })
    }

    /// Records that the ROS 2 spin loop completed an iteration.
    #[inline]
    pub fn update_spin_heartbeat(&self) {
        self.spin_heartbeat_ms
            .store(monotonic_ms(), Ordering::Relaxed);
    }
}

/// Evaluates if a cached message is fresh enough to replay to a newly subscribed client.
#[must_use]
pub fn fresh_cached_message(
    topic: &str,
    cached: Option<CachedMessage>,
    now: Instant,
) -> Option<Arc<str>> {
    let max_age = match topic {
        "/robot_pose" | "/tf" => Some(Duration::from_secs(2)),
        "/map" => None,
        _ => Some(Duration::ZERO),
    };
    cached.and_then(|entry| {
        if max_age.is_none_or(|age| now.duration_since(entry.received_at) <= age) {
            Some(entry.json_payload)
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_cache_does_not_expire() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"map\":true}"),
            received_at: now - Duration::from_secs(3600),
        };
        assert!(fresh_cached_message("/map", Some(cached), now).is_some());
    }

    #[test]
    fn test_fresh_pose_cache_is_replayed() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"x\":1.0,\"y\":2.0}"),
            received_at: now - Duration::from_millis(500),
        };
        let result = fresh_cached_message("/robot_pose", Some(cached), now);
        assert!(result.is_some());
        assert_eq!(&*result.unwrap(), "{\"x\":1.0,\"y\":2.0}");
    }

    #[test]
    fn test_stale_pose_cache_is_not_replayed() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"pose\":true}"),
            received_at: now - Duration::from_millis(2001),
        };
        assert!(fresh_cached_message("/robot_pose", Some(cached), now).is_none());
    }

    #[test]
    fn test_fresh_tf_cache_is_replayed() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"transforms\":[]}"),
            received_at: now - Duration::from_millis(1500),
        };
        assert!(fresh_cached_message("/tf", Some(cached), now).is_some());
    }

    #[test]
    fn test_stale_tf_cache_is_not_replayed() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"transforms\":[]}"),
            received_at: now - Duration::from_millis(2500),
        };
        assert!(fresh_cached_message("/tf", Some(cached), now).is_none());
    }

    #[test]
    fn test_unknown_topic_cache_expires_immediately() {
        let now = Instant::now();
        let cached = CachedMessage {
            json_payload: Arc::from("{\"data\":123}"),
            received_at: now,
        };
        assert!(
            fresh_cached_message("/unknown_topic", Some(cached), now + Duration::from_millis(1))
                .is_none()
        );
    }

    #[test]
    fn test_none_cache_returns_none() {
        let now = Instant::now();
        assert!(fresh_cached_message("/map", None, now).is_none());
        assert!(fresh_cached_message("/robot_pose", None, now).is_none());
    }

    #[test]
    fn test_rate_limiter_allows_first_and_due_messages() {
        let now = Instant::now();
        let mut limiter = RateLimiter::new(Duration::from_millis(100));
        assert!(limiter.should_process(now));
        assert!(!limiter.should_process(now + Duration::from_millis(50)));
        assert!(!limiter.should_process(now + Duration::from_millis(99)));
        assert!(limiter.should_process(now + Duration::from_millis(100)));
        assert!(!limiter.should_process(now + Duration::from_millis(150)));
        assert!(limiter.should_process(now + Duration::from_millis(200)));
    }

    #[test]
    fn test_rate_limiter_zero_duration_always_allows() {
        let now = Instant::now();
        let mut limiter = RateLimiter::new(Duration::ZERO);
        assert!(limiter.should_process(now));
        assert!(limiter.should_process(now));
        assert!(limiter.should_process(now));
    }

    #[test]
    fn test_rate_limiter_skips_rapid_bursts() {
        let now = Instant::now();
        let mut limiter = RateLimiter::new(Duration::from_millis(50));
        assert!(limiter.should_process(now));
        for ms in 1..50 {
            assert!(!limiter.should_process(now + Duration::from_millis(ms)));
        }
        assert!(limiter.should_process(now + Duration::from_millis(50)));
    }

    #[test]
    fn test_subscription_key_equality_and_hash() {
        let key1 = SubscriptionKey::new("/battery", Some("1".to_string()));
        let key2 = SubscriptionKey::new("/battery", Some("1".to_string()));
        let key3 = SubscriptionKey::new("/battery", None);
        let key4 = SubscriptionKey::new("/charger", Some("1".to_string()));

        assert_eq!(key1, key2);
        assert_ne!(key1, key3);
        assert_ne!(key1, key4);
    }

    #[test]
    fn test_monotonic_ms_is_non_decreasing() {
        let t1 = monotonic_ms();
        std::thread::sleep(Duration::from_millis(5));
        let t2 = monotonic_ms();
        assert!(t2 >= t1);
    }

    #[test]
    fn test_cached_message_creation_and_clone() {
        let payload: Arc<str> = Arc::from("{\"test\":123}");
        let cached1 = CachedMessage::new(payload.clone());
        let cached2 = cached1.clone();

        assert_eq!(&*cached1.json_payload, "{\"test\":123}");
        assert_eq!(&*cached2.json_payload, "{\"test\":123}");
        assert_eq!(cached1.received_at, cached2.received_at);
        assert!(Arc::ptr_eq(&cached1.json_payload, &cached2.json_payload));
    }
}
