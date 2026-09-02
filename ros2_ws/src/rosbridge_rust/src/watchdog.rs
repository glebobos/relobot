//! Watchdog monitoring task ensuring ROS 2 spin loop liveness and system health.

use crate::state::{monotonic_ms, SharedState};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;
use tracing::{info, warn};

/// Runs the periodic health check watchdog.
///
/// Emits warnings if the ROS 2 background spin thread stalls and periodically logs health statistics.
pub async fn run_watchdog(state: Arc<SharedState>) {
    let mut check_interval = tokio::time::interval(Duration::from_secs(2));
    let mut report_counter: u32 = 0;

    loop {
        check_interval.tick().await;

        let now = monotonic_ms();
        let last_spin = state.spin_heartbeat_ms.load(Ordering::Relaxed);
        let spin_diff_ms = now.saturating_sub(last_spin);

        if spin_diff_ms > 5000 {
            warn!(
                spin_diff_ms,
                "ROS 2 spin thread stalled! Last tick was {}ms ago", spin_diff_ms
            );
        }

        report_counter += 1;
        if report_counter >= 15 {
            report_counter = 0;
            let active = state.active_clients.load(Ordering::Relaxed);
            info!(
                active_clients = active,
                spin_tick_diff_ms = spin_diff_ms,
                "Watchdog Health Check: bridge operating normally"
            );
        }
    }
}
