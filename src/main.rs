mod demo;
mod errors;
mod mavlink_listener;
mod mission;
mod state;
mod telemetry;
mod ws;

use axum::{Json, Router, routing::{get, post}};
use serde_json::{Value, json};
use state::AppState;
use std::{env, net::SocketAddr, path::PathBuf, time::{SystemTime, UNIX_EPOCH}};
use telemetry::DataSource;
use tower_http::{services::ServeDir, trace::TraceLayer};
use tracing::info;
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("groundlink=info,tower_http=info")),
        )
        .init();

    let demo_mode = env::args().any(|arg| arg == "--demo");
    let source = if demo_mode { DataSource::Demo } else { DataSource::Sitl };
    let state = AppState::new(256, source);

    if demo_mode {
        info!("starting GroundLink in native demo mode");
        tokio::spawn(demo::run(state.clone()));
    } else {
        tokio::spawn(mavlink_listener::run(state.clone()));
    }

    let static_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("static");

    let app = Router::new()
        .route("/health", get(health))
        .route("/luck", post(try_luck))
        .route("/api/mission", post(mission::upload_handler))
        .route("/ws", get(ws::ws_handler))
        .fallback_service(ServeDir::new(static_dir).append_index_html_on_directories(true))
        .layer(TraceLayer::new_for_http())
        .with_state(state);

    let address = SocketAddr::from(([127, 0, 0, 1], 3000));
    let listener = tokio::net::TcpListener::bind(address)
        .await
        .expect("failed to bind GroundLink HTTP server");

    info!(%address, demo_mode, "GroundLink online");
    axum::serve(listener, app).await.expect("server failed");
}

async fn health() -> Json<Value> {
    Json(json!({
        "status": "ok",
        "service": "groundlink"
    }))
}

async fn try_luck() -> Json<Value> {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let roll = (nanos % 10) as u8 + 1;
    let shutdown = roll == 1;

    if shutdown {
        tokio::spawn(async {
            tokio::time::sleep(std::time::Duration::from_millis(1800)).await;
            tracing::warn!("TRY YOUR LUCK rolled a 1/10 ◈ opsi... GroundLink exiting");
            std::process::exit(0);
        });
    }

    Json(json!({
        "roll": roll,
        "shutdown": shutdown
    }))
}
