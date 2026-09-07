use crate::{
    errors::GroundLinkError,
    mission::MissionEvent,
    state::{AppState, SharedMavlinkConnection},
    telemetry::{DataSource, ServerMessage},
};
use mavlink::{dialects::ardupilotmega::MavMessage, AsyncMavConnection};
use std::{sync::Arc, time::Duration};
use tokio::time::sleep;
use tracing::{info, warn};

const MAVLINK_ENDPOINT: &str = "udpin:0.0.0.0:14550";
const MAX_BACKOFF_SECS: u64 = 10;

pub async fn run(state: AppState) {
    let mut backoff_secs = 1_u64;

    loop {
        info!(endpoint = MAVLINK_ENDPOINT, "connecting to ArduPilot SITL");

        match connect().await {
            Ok(connection) => {
                let connection: SharedMavlinkConnection = Arc::from(connection);
                *state.mavlink_connection.write().await = Some(connection.clone());
                set_connection_status(&state, true).await;
                backoff_secs = 1;

                if let Err(error) = read_loop(connection, &state).await {
                    warn!(%error, "MAVLink receive loop ended");
                }

                *state.mavlink_connection.write().await = None;
                set_connection_status(&state, false).await;
            }
            Err(error) => {
                warn!(%error, "could not connect to SITL");
                *state.mavlink_connection.write().await = None;
                set_connection_status(&state, false).await;
            }
        }

        sleep(Duration::from_secs(backoff_secs)).await;
        backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
    }
}

async fn connect(
) -> Result<Box<dyn AsyncMavConnection<MavMessage> + Sync + Send>, GroundLinkError> {
    mavlink::connect_async::<MavMessage>(MAVLINK_ENDPOINT)
        .await
        .map_err(|error| GroundLinkError::MavlinkConnect(error.to_string()))
}

async fn read_loop(
    connection: SharedMavlinkConnection,
    state: &AppState,
) -> Result<(), String> {
    let mut telemetry = state.latest.read().await.clone().unwrap_or_default();

    loop {
        match connection.recv().await {
            Ok((header, message)) => {
                if matches!(message, MavMessage::HEARTBEAT(_)) {
                    *state.vehicle_target.write().await = Some((header.system_id, header.component_id));
                }

                match &message {
                    MavMessage::MISSION_REQUEST_INT(data) => {
                        let _ = state.mission_events.send(MissionEvent::RequestInt(data.seq));
                    }
                    MavMessage::MISSION_REQUEST(data) => {
                        let _ = state.mission_events.send(MissionEvent::Request(data.seq));
                    }
                    MavMessage::MISSION_ACK(data) => {
                        let _ = state.mission_events.send(MissionEvent::Ack(data.mavtype));
                    }
                    _ => {}
                }

                if telemetry.apply_mavlink(&message) {
                    *state.latest.write().await = Some(telemetry.clone());
                    let _ = state.tx.send(ServerMessage::Telemetry {
                        data: telemetry.clone(),
                    });
                }
            }
            Err(error) => return Err(error.to_string()),
        }
    }
}

async fn set_connection_status(state: &AppState, connected: bool) {
    *state.source.write().await = DataSource::Sitl;

    let mut current = state.connected.write().await;
    if *current == connected {
        return;
    }

    *current = connected;
    let _ = state.tx.send(ServerMessage::Status {
        connected,
        source: DataSource::Sitl,
    });
}
