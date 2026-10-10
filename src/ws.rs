use crate::{state::AppState, telemetry::ServerMessage};
use axum::{
    extract::{
        State,
        ws::{Message, WebSocket, WebSocketUpgrade},
    },
    response::IntoResponse,
};
use tokio::sync::broadcast::error::RecvError;
use tracing::debug;

pub async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(mut socket: WebSocket, state: AppState) {
    let mut rx = state.tx.subscribe();

    let connected = *state.connected.read().await;
    let source = *state.source.read().await;
    if send_json(&mut socket, &ServerMessage::Status { connected, source })
        .await
        .is_err()
    {
        return;
    }

    let mut initial = Vec::new();
    if let Some(latest) = state.latest.read().await.clone() {
        initial.push(ServerMessage::Telemetry { data: latest });
    }
    initial.push(ServerMessage::Home {
        home: state.home.read().await.clone(),
    });
    if let Some(plan) = state.mission.read().await.clone() {
        initial.push(ServerMessage::Mission { plan });
    }
    if let Some(plan) = state.sweep.read().await.clone() {
        initial.push(ServerMessage::Sweep { plan: Some(plan) });
        if let Some(progress) = state.sweep_progress.read().await.clone() {
            initial.push(ServerMessage::SweepProgress { progress });
        }
    }
    for message in initial {
        if send_json(&mut socket, &message).await.is_err() {
            return;
        }
    }

    loop {
        tokio::select! {
            broadcast = rx.recv() => {
                match broadcast {
                    Ok(message) => {
                        if send_json(&mut socket, &message).await.is_err() {
                            break;
                        }
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        debug!(skipped, "websocket client lagged; dropping stale frames");
                        continue;
                    }
                    Err(RecvError::Closed) => break,
                }
            }
            incoming = socket.recv() => {
                match incoming {
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(_)) => break,
                    _ => {}
                }
            }
        }
    }
}

async fn send_json(socket: &mut WebSocket, message: &ServerMessage) -> Result<(), ()> {
    let json = serde_json::to_string(message).map_err(|_| ())?;
    socket
        .send(Message::Text(json.into()))
        .await
        .map_err(|_| ())
}
