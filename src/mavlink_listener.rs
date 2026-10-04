use crate::{
    errors::GroundLinkError,
    mission::{GCS_COMPONENT, GCS_SYSTEM, HomePosition, MissionEvent, gcs_header},
    state::{AppState, SharedMavlinkConnection},
    telemetry::{DataSource, ServerMessage},
};
use mavlink::{
    AsyncMavConnection,
    dialects::ardupilotmega::{
        COMMAND_LONG_DATA, MavAutopilot, MavCmd, MavMessage, MavMissionType, MavType,
    },
};
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
                set_connection_status(&state, false).await;
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

async fn connect() -> Result<Box<dyn AsyncMavConnection<MavMessage> + Sync + Send>, GroundLinkError>
{
    mavlink::connect_async::<MavMessage>(MAVLINK_ENDPOINT)
        .await
        .map_err(|error| GroundLinkError::MavlinkConnect(error.to_string()))
}

async fn read_loop(connection: SharedMavlinkConnection, state: &AppState) -> Result<(), String> {
    let mut telemetry = state.latest.read().await.clone().unwrap_or_default();

    let mut last_home_request = tokio::time::Instant::now() - Duration::from_secs(10);
    let mut last_heartbeat = tokio::time::Instant::now();
    loop {
        if last_heartbeat.elapsed() >= Duration::from_secs(10) {
            return Err("vehicle heartbeat timed out".into());
        }
        let received = tokio::time::timeout(
            Duration::from_secs(10).saturating_sub(last_heartbeat.elapsed()),
            connection.recv(),
        )
        .await
        .map_err(|_| "vehicle heartbeat timed out".to_string())?;
        match received {
            Ok((header, message)) => {
                if let MavMessage::HEARTBEAT(data) = &message {
                    if data.autopilot != MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
                        || header.component_id != 1
                        || !matches!(
                            data.mavtype,
                            MavType::MAV_TYPE_QUADROTOR
                                | MavType::MAV_TYPE_HEXAROTOR
                                | MavType::MAV_TYPE_OCTOROTOR
                                | MavType::MAV_TYPE_TRICOPTER
                                | MavType::MAV_TYPE_HELICOPTER
                                | MavType::MAV_TYPE_COAXIAL
                                | MavType::MAV_TYPE_DODECAROTOR
                        )
                    {
                        continue;
                    }
                    let mut target = state.vehicle_target.write().await;
                    if target.is_none() {
                        *target = Some((header.system_id, header.component_id));
                    }
                }
                if *state.vehicle_target.read().await
                    != Some((header.system_id, header.component_id))
                {
                    continue;
                }
                if matches!(message, MavMessage::HEARTBEAT(_)) {
                    last_heartbeat = tokio::time::Instant::now();
                    set_connection_status(state, true).await;
                }
                if matches!(message, MavMessage::HEARTBEAT(_))
                    && last_home_request.elapsed() >= Duration::from_secs(5)
                {
                    // HOME_POSITION is vehicle-provided, including updates after arming.
                    let request = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                        target_system: header.system_id,
                        target_component: header.component_id,
                        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                        param1: 242.0,
                        ..Default::default()
                    });
                    let _ = connection.send(&gcs_header(), &request).await;
                    last_home_request = tokio::time::Instant::now();
                }
                let addressed_to_us = |system, component| {
                    (system == 0 || system == GCS_SYSTEM)
                        && (component == 0 || component == GCS_COMPONENT)
                };
                #[allow(deprecated)] // Older vehicles may still request using MISSION_REQUEST.
                match &message {
                    MavMessage::MISSION_REQUEST_INT(data)
                        if data.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION
                            && addressed_to_us(data.target_system, data.target_component) =>
                    {
                        let _ = state
                            .mission_events
                            .send(MissionEvent::RequestInt(data.seq));
                    }
                    MavMessage::MISSION_REQUEST(data)
                        if data.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION
                            && addressed_to_us(data.target_system, data.target_component) =>
                    {
                        let _ = state.mission_events.send(MissionEvent::Request(data.seq));
                    }
                    MavMessage::MISSION_ACK(data)
                        if data.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION
                            && addressed_to_us(data.target_system, data.target_component) =>
                    {
                        let _ = state.mission_events.send(MissionEvent::Ack(data.mavtype));
                    }
                    MavMessage::PARAM_VALUE(data) => {
                        let _ = state.mission_events.send(MissionEvent::Parameter(
                            data.param_id.to_str().unwrap_or("").to_string(),
                            data.param_value,
                        ));
                    }
                    MavMessage::HOME_POSITION(data) => {
                        let home = HomePosition {
                            lat: data.latitude as f64 / 1e7,
                            lon: data.longitude as f64 / 1e7,
                            alt_m: data.altitude as f32 / 1000.0,
                        };
                        if (-90.0..=90.0).contains(&home.lat)
                            && (-180.0..=180.0).contains(&home.lon)
                        {
                            *state.home.write().await = Some(home.clone());
                            let _ = state.tx.send(ServerMessage::Home { home: Some(home) });
                        }
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
    if !connected {
        *state.vehicle_target.write().await = None;
        *state.home.write().await = None;
        let _ = state.tx.send(ServerMessage::Home { home: None });
    }

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
