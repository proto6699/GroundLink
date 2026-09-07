use crate::{state::AppState, telemetry::{DataSource, ServerMessage}};
use axum::{extract::State, http::StatusCode, Json};
use mavlink::{
    dialects::ardupilotmega::{
        MavCmd, MavFrame, MavMessage, MavMissionResult, MavMissionType,
        MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA,
    },
    MavHeader,
};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tokio::{sync::broadcast::error::RecvError, time::{timeout, Duration, Instant}};

const MISSION_TIMEOUT: Duration = Duration::from_secs(10);
const MAX_WAYPOINTS: usize = 50;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionWaypoint {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f32,
}

#[derive(Debug, Deserialize)]
pub struct MissionUploadRequest {
    pub waypoints: Vec<MissionWaypoint>,
}

#[derive(Debug, Clone)]
pub enum MissionEvent {
    RequestInt(u16),
    Request(u16),
    Ack(MavMissionResult),
}

pub async fn upload_handler(
    State(state): State<AppState>,
    Json(payload): Json<MissionUploadRequest>,
) -> (StatusCode, Json<Value>) {
    if let Err(error) = validate_waypoints(&payload.waypoints) {
        return (StatusCode::BAD_REQUEST, Json(json!({"ok": false, "error": error})));
    }

    let source = *state.source.read().await;
    let result = match source {
        DataSource::Demo => Ok("demo mission loaded — tiny guy will follow it".to_string()),
        DataSource::Sitl => upload_to_vehicle(&state, &payload.waypoints).await,
    };

    match result {
        Ok(message) => {
            *state.mission.write().await = payload.waypoints.clone();
            let _ = state.tx.send(ServerMessage::Mission {
                waypoints: payload.waypoints.clone(),
            });
            (StatusCode::OK, Json(json!({
                "ok": true,
                "message": message,
                "count": payload.waypoints.len(),
            })))
        }
        Err(error) => (
            StatusCode::BAD_GATEWAY,
            Json(json!({"ok": false, "error": error})),
        ),
    }
}

fn validate_waypoints(waypoints: &[MissionWaypoint]) -> Result<(), String> {
    if waypoints.is_empty() {
        return Err("mission needs at least one waypoint".into());
    }
    if waypoints.len() > MAX_WAYPOINTS {
        return Err(format!("mission is capped at {MAX_WAYPOINTS} waypoints in v1"));
    }

    for (index, wp) in waypoints.iter().enumerate() {
        if !wp.lat.is_finite() || !(-90.0..=90.0).contains(&wp.lat) {
            return Err(format!("waypoint {} has an invalid latitude", index + 1));
        }
        if !wp.lon.is_finite() || !(-180.0..=180.0).contains(&wp.lon) {
            return Err(format!("waypoint {} has an invalid longitude", index + 1));
        }
        if !wp.alt_m.is_finite() || !(1.0..=500.0).contains(&wp.alt_m) {
            return Err(format!("waypoint {} altitude must be 1–500 m", index + 1));
        }
    }

    Ok(())
}

async fn upload_to_vehicle(state: &AppState, waypoints: &[MissionWaypoint]) -> Result<String, String> {
    if !*state.connected.read().await {
        return Err("SITL is not connected".into());
    }

    let connection = state
        .mavlink_connection
        .read()
        .await
        .clone()
        .ok_or_else(|| "MAVLink connection is not ready".to_string())?;

    let (target_system, target_component) = state
        .vehicle_target
        .read()
        .await
        .ok_or_else(|| "waiting for a vehicle heartbeat before mission upload".to_string())?;

    let mut events = state.mission_events.subscribe();
    let header = MavHeader::default();
    let mission_type = MavMissionType::MAV_MISSION_TYPE_MISSION;

    let count = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        target_system,
        target_component,
        count: waypoints.len() as u16,
        mission_type,
        ..Default::default()
    });

    connection
        .send(&header, &count)
        .await
        .map_err(|error| format!("failed to announce mission: {error}"))?;

    let deadline = Instant::now() + MISSION_TIMEOUT;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err("mission upload timed out waiting for ArduPilot".into());
        }

        let event = match timeout(remaining, events.recv()).await {
            Ok(Ok(event)) => event,
            Ok(Err(RecvError::Lagged(_))) => continue,
            Ok(Err(RecvError::Closed)) => return Err("mission protocol channel closed".into()),
            Err(_) => return Err("mission upload timed out waiting for ArduPilot".into()),
        };

        match event {
            MissionEvent::RequestInt(seq) | MissionEvent::Request(seq) => {
                let waypoint = waypoints
                    .get(seq as usize)
                    .ok_or_else(|| format!("vehicle requested invalid mission sequence {seq}"))?;

                let item = MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
                    param1: 0.0,
                    param2: 2.0,
                    param3: 0.0,
                    param4: f32::NAN,
                    x: (waypoint.lat * 10_000_000.0).round() as i32,
                    y: (waypoint.lon * 10_000_000.0).round() as i32,
                    z: waypoint.alt_m,
                    seq,
                    command: MavCmd::MAV_CMD_NAV_WAYPOINT,
                    target_system,
                    target_component,
                    frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    current: if seq == 0 { 1 } else { 0 },
                    autocontinue: 1,
                    mission_type,
                });

                connection
                    .send(&header, &item)
                    .await
                    .map_err(|error| format!("failed to send waypoint {}: {error}", seq + 1))?;
            }
            MissionEvent::Ack(result) => {
                if result == MavMissionResult::MAV_MISSION_ACCEPTED {
                    return Ok(format!("ArduPilot accepted {} waypoint(s)", waypoints.len()));
                }
                return Err(format!("ArduPilot rejected mission: {result:?}"));
            }
        }
    }
}
