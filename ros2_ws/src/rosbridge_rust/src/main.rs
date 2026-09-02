//! Binary entrypoint for the `rosbridge_rust` server.

use rosbridge_rust::error::Result;
use rosbridge_rust::{handle_client_connection, init_ros2_node, run_watchdog, SharedState};
use std::net::SocketAddr;
use std::sync::atomic::Ordering;
use tokio::net::TcpListener;
use tokio::sync::broadcast;
use tokio_util::sync::CancellationToken;
use tracing::{error, info, warn};
use tracing_subscriber::filter::LevelFilter;
use tracing_subscriber::EnvFilter;

const DEFAULT_PORT: u16 = 9090;
const UPSTREAM_URL: &str = "ws://127.0.0.1:9091";

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize structured tracing logger
    let filter = EnvFilter::builder()
        .with_default_directive(LevelFilter::INFO.into())
        .from_env_lossy();

    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(false)
        .init();

    info!("Starting rosbridge_rust server...");

    // Create broadcast channels for high-bandwidth native topics
    let (tf_tx, _) = broadcast::channel(100);
    let (map_tx, _) = broadcast::channel(10);
    let (pose_tx, _) = broadcast::channel(100);

    let state = SharedState::new(tf_tx, map_tx, pose_tx);

    // Initialize ROS 2 node and native topic subscribers
    let ros_tasks = init_ros2_node(state.clone())?;

    // Spawn watchdog monitor task
    let state_watchdog = state.clone();
    tokio::spawn(async move {
        run_watchdog(state_watchdog).await;
    });

    // Monitor subscriber task lifecycle
    tokio::spawn(async move {
        tokio::select! {
            res = ros_tasks.tf_task => error!("TF subscriber task exited: {:?}", res),
            res = ros_tasks.map_task => error!("Map subscriber task exited: {:?}", res),
            res = ros_tasks.pose_task => error!("Robot pose subscriber task exited: {:?}", res),
        }
    });

    // Bind TCP WebSocket server
    let bind_addr: SocketAddr = ([0, 0, 0, 0], DEFAULT_PORT).into();
    let listener = TcpListener::bind(&bind_addr).await?;
    info!(addr = %bind_addr, "WebSocket server listening");

    // Graceful shutdown token
    let shutdown_token = CancellationToken::new();
    let shutdown_token_clone = shutdown_token.clone();

    tokio::spawn(async move {
        if let Ok(()) = tokio::signal::ctrl_c().await {
            info!("Shutdown signal received; initiating graceful termination...");
            shutdown_token_clone.cancel();
        }
    });

    loop {
        tokio::select! {
            _ = shutdown_token.cancelled() => {
                info!("Server stopped accepting new connections");
                break;
            }
            accept_res = listener.accept() => {
                match accept_res {
                    Ok((stream, client_addr)) => {
                        let state_clone = state.clone();
                        let token = shutdown_token.clone();

                        tokio::spawn(async move {
                            let count = state_clone.active_clients.fetch_add(1, Ordering::SeqCst) + 1;
                            info!(client = %client_addr, total_active = count, "Client connected");

                            tokio::select! {
                                _ = token.cancelled() => {
                                    debug_client_disconnect(client_addr);
                                }
                                res = handle_client_connection(stream, client_addr, state_clone.clone(), UPSTREAM_URL) => {
                                    if let Err(e) = res {
                                        warn!(client = %client_addr, error = %e, "Client session terminated with error");
                                    }
                                }
                            }

                            let count = state_clone.active_clients.fetch_sub(1, Ordering::SeqCst) - 1;
                            info!(client = %client_addr, total_active = count, "Client disconnected");
                        });
                    }
                    Err(e) => {
                        error!(error = %e, "TCP accept failed");
                    }
                }
            }
        }
    }

    info!("rosbridge_rust shutdown complete.");
    Ok(())
}

#[inline]
fn debug_client_disconnect(client_addr: SocketAddr) {
    info!(client = %client_addr, "Closing client connection on shutdown");
}
