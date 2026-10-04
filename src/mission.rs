use crate::{
    state::AppState,
    telemetry::{DataSource, ServerMessage},
};
use axum::{Json, extract::State, http::StatusCode};
use mavlink::{
    MavHeader,
    dialects::ardupilotmega::{
        MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA, MavCmd, MavFrame, MavMessage, MavMissionResult,
        MavMissionType, MavParamType, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    },
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::{
    sync::broadcast::error::RecvError,
    time::{Duration, Instant, timeout},
};

const MISSION_TIMEOUT: Duration = Duration::from_secs(10);
const MAX_WAYPOINTS: usize = 50;
pub const GCS_SYSTEM: u8 = 255;
pub const GCS_COMPONENT: u8 = 190;

pub fn gcs_header() -> MavHeader {
    MavHeader {
        system_id: GCS_SYSTEM,
        component_id: GCS_COMPONENT,
        ..Default::default()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MissionWaypoint {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct HomePosition {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f32,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MissionKind {
    #[default]
    Waypoints,
    Orbit,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct OrbitSettings {
    pub diameter_m: f32,
    pub alt_m: f32,
}

fn default_speed() -> f32 {
    5.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionUploadRequest {
    #[serde(default)]
    pub kind: MissionKind,
    #[serde(default)]
    pub waypoints: Vec<MissionWaypoint>,
    #[serde(default = "default_speed")]
    pub speed_m_s: f32,
    pub orbit: Option<OrbitSettings>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MissionPlan {
    #[serde(flatten)]
    pub settings: MissionUploadRequest,
    pub home: HomePosition,
}

#[derive(Debug, Clone)]
pub enum MissionEvent {
    RequestInt(u16),
    Request(u16),
    Ack(MavMissionResult),
    Parameter(String, f32),
}

pub async fn upload_handler(
    State(state): State<AppState>,
    Json(payload): Json<MissionUploadRequest>,
) -> (StatusCode, Json<Value>) {
    if let Err(error) = validate_plan(&payload) {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({"ok":false,"error":error})),
        );
    }
    // One writer owns both the parameter exchange and the mission protocol.
    let Ok(_upload) = state.mission_upload.try_lock() else {
        return (
            StatusCode::CONFLICT,
            Json(json!({"ok":false,"error":"another mission upload is in progress"})),
        );
    };
    let Some(home) = state.home.read().await.clone() else {
        return (
            StatusCode::CONFLICT,
            Json(
                json!({"ok":false,"error":"waiting for vehicle HOME_POSITION; home is never inferred from the moving drone"}),
            ),
        );
    };
    let plan = MissionPlan {
        settings: payload,
        home,
    };
    let source = *state.source.read().await;
    let result = match source {
        DataSource::Demo => Ok("demo flight plan loaded".to_string()),
        DataSource::Sitl => upload_to_vehicle(&state, &plan).await,
    };
    match result {
        Ok(message) => {
            *state.mission.write().await = Some(plan.clone());
            let _ = state.tx.send(ServerMessage::Mission { plan: plan.clone() });
            (
                StatusCode::OK,
                Json(
                    json!({"ok":true,"message":message,"count":plan.settings.waypoints.len(),"plan":plan}),
                ),
            )
        }
        Err(error) => (
            StatusCode::BAD_GATEWAY,
            Json(json!({"ok":false,"error":error})),
        ),
    }
}

fn validate_plan(plan: &MissionUploadRequest) -> Result<(), String> {
    if !plan.speed_m_s.is_finite() || !(0.5..=20.0).contains(&plan.speed_m_s) {
        return Err("speed must be 0.5–20 m/s".into());
    }
    match plan.kind {
        MissionKind::Orbit => {
            if !plan.waypoints.is_empty() {
                return Err("home orbit cannot include waypoints".into());
            }
            let orbit = plan
                .orbit
                .as_ref()
                .ok_or("home orbit needs diameter and altitude")?;
            // Whole metre radii <=255 are stored exactly by ArduPilot's mission format.
            if !orbit.diameter_m.is_finite()
                || !(10.0..=500.0).contains(&orbit.diameter_m)
                || orbit.diameter_m % 2.0 != 0.0
            {
                return Err("orbit diameter must be 10–500 m in 2 m steps".into());
            }
            validate_altitude(orbit.alt_m)?;
            if orbit_rate(plan) > 90.0 {
                return Err(
                    "speed is too high for this diameter; orbit rate must be at most 90°/s".into(),
                );
            }
        }
        MissionKind::Waypoints => {
            if plan.orbit.is_some() {
                return Err("waypoint mission cannot include orbit settings".into());
            }
            if plan.waypoints.is_empty() {
                return Err("mission needs at least one waypoint".into());
            }
            if plan.waypoints.len() > MAX_WAYPOINTS {
                return Err(format!("mission is capped at {MAX_WAYPOINTS} waypoints"));
            }
            for (index, wp) in plan.waypoints.iter().enumerate() {
                if !wp.lat.is_finite() || !(-90.0..=90.0).contains(&wp.lat) {
                    return Err(format!("waypoint {} has an invalid latitude", index + 1));
                }
                if !wp.lon.is_finite() || !(-180.0..=180.0).contains(&wp.lon) {
                    return Err(format!("waypoint {} has an invalid longitude", index + 1));
                }
                validate_altitude(wp.alt_m).map_err(|e| format!("waypoint {}: {e}", index + 1))?;
            }
        }
    }
    Ok(())
}

fn validate_altitude(alt_m: f32) -> Result<(), String> {
    if !alt_m.is_finite() || !(1.0..=500.0).contains(&alt_m) {
        return Err("altitude must be 1–500 m above home".into());
    }
    Ok(())
}

fn orbit_rate(plan: &MissionUploadRequest) -> f32 {
    (plan.speed_m_s / (plan.orbit.as_ref().unwrap().diameter_m / 2.0)).to_degrees()
}

fn mission_items(
    plan: &MissionPlan,
    target_system: u8,
    target_component: u8,
) -> Vec<MISSION_ITEM_INT_DATA> {
    let base = MISSION_ITEM_INT_DATA {
        target_system,
        target_component,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        autocontinue: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        ..Default::default()
    };
    let altitude = plan
        .settings
        .orbit
        .as_ref()
        .map(|o| o.alt_m)
        .unwrap_or_else(|| plan.settings.waypoints[0].alt_m);
    // ArduPilot reserves sequence zero for home. Do not lose the first user waypoint.
    let mut items = vec![
        MISSION_ITEM_INT_DATA {
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            frame: MavFrame::MAV_FRAME_GLOBAL,
            x: (plan.home.lat * 1e7).round() as i32,
            y: (plan.home.lon * 1e7).round() as i32,
            z: plan.home.alt_m,
            ..base.clone()
        },
        MISSION_ITEM_INT_DATA {
            command: MavCmd::MAV_CMD_NAV_TAKEOFF,
            z: altitude,
            param4: f32::NAN,
            ..base.clone()
        },
        MISSION_ITEM_INT_DATA {
            command: MavCmd::MAV_CMD_DO_CHANGE_SPEED,
            param1: 1.0,
            param2: plan.settings.speed_m_s,
            param3: -1.0,
            ..base.clone()
        },
    ];
    match plan.settings.kind {
        MissionKind::Waypoints => {
            for wp in &plan.settings.waypoints {
                items.push(MISSION_ITEM_INT_DATA {
                    command: MavCmd::MAV_CMD_NAV_WAYPOINT,
                    param2: 2.0,
                    param4: f32::NAN,
                    x: (wp.lat * 1e7).round() as i32,
                    y: (wp.lon * 1e7).round() as i32,
                    z: wp.alt_m,
                    ..base.clone()
                });
            }
        }
        MissionKind::Orbit => {
            let orbit = plan.settings.orbit.as_ref().unwrap();
            items.push(MISSION_ITEM_INT_DATA {
                command: MavCmd::MAV_CMD_NAV_LOITER_TURNS,
                param1: 1.0,
                param3: orbit.diameter_m / 2.0,
                x: (plan.home.lat * 1e7).round() as i32,
                y: (plan.home.lon * 1e7).round() as i32,
                z: orbit.alt_m,
                ..base.clone()
            });
            // Repeat only the orbit, not takeoff. A mode change (e.g. RTL) exits it.
            items.push(MISSION_ITEM_INT_DATA {
                command: MavCmd::MAV_CMD_DO_JUMP,
                param1: 3.0,
                param2: -1.0,
                ..base
            });
        }
    }
    for (seq, item) in items.iter_mut().enumerate() {
        item.seq = seq as u16;
    }
    items
}

async fn circle_parameter(state: &AppState, value: Option<f32>) -> Result<f32, String> {
    let connection = state
        .mavlink_connection
        .read()
        .await
        .clone()
        .ok_or("MAVLink connection is not ready")?;
    let (target_system, target_component) = state
        .vehicle_target
        .read()
        .await
        .ok_or("waiting for vehicle heartbeat")?;
    let mut events = state.mission_events.subscribe();
    let param_id = "CIRCLE_RATE".into();
    let message = if let Some(param_value) = value {
        MavMessage::PARAM_SET(PARAM_SET_DATA {
            target_system,
            target_component,
            param_id,
            param_value,
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        })
    } else {
        MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            target_system,
            target_component,
            param_id,
            param_index: -1,
        })
    };
    for _ in 0..3 {
        connection
            .send(&gcs_header(), &message)
            .await
            .map_err(|e| format!("CIRCLE_RATE exchange failed: {e}"))?;
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            match timeout(
                deadline.saturating_duration_since(Instant::now()),
                events.recv(),
            )
            .await
            {
                Ok(Ok(MissionEvent::Parameter(name, actual))) if name == "CIRCLE_RATE" => {
                    if value.is_some_and(|expected| (actual - expected).abs() > 0.01) {
                        return Err(format!(
                            "CIRCLE_RATE readback mismatch: requested {value:?}, got {actual}"
                        ));
                    }
                    if !actual.is_finite() {
                        return Err("invalid CIRCLE_RATE readback".into());
                    }
                    return Ok(actual);
                }
                Ok(Ok(_)) | Ok(Err(RecvError::Lagged(_))) => continue,
                _ => break,
            }
        }
    }
    Err("CIRCLE_RATE was not confirmed by ArduPilot".into())
}

async fn upload_to_vehicle(state: &AppState, plan: &MissionPlan) -> Result<String, String> {
    if !*state.connected.read().await {
        return Err("vehicle is not connected".into());
    }
    if plan.settings.kind == MissionKind::Orbit
        && state.latest.read().await.as_ref().is_some_and(|t| t.armed)
    {
        return Err(
            "disarm before uploading a home orbit; upload also configures CIRCLE_RATE".into(),
        );
    }
    let previous_rate = if plan.settings.kind == MissionKind::Orbit {
        let previous = circle_parameter(state, None).await?;
        if let Err(error) = circle_parameter(state, Some(orbit_rate(&plan.settings))).await {
            let rollback = circle_parameter(state, Some(previous)).await;
            return Err(format!("{error}; CIRCLE_RATE restore: {rollback:?}"));
        }
        Some(previous)
    } else {
        None
    };
    match (transfer_mission(state, plan).await, previous_rate) {
        (Err(upload_error), Some(previous)) => {
            match circle_parameter(state, Some(previous)).await {
                Ok(_) => Err(upload_error),
                Err(error) => Err(format!(
                    "{upload_error}; could not restore CIRCLE_RATE: {error}"
                )),
            }
        }
        (result, _) => result,
    }
}

async fn transfer_mission(state: &AppState, plan: &MissionPlan) -> Result<String, String> {
    let connection = state
        .mavlink_connection
        .read()
        .await
        .clone()
        .ok_or("MAVLink connection is not ready")?;
    let (target_system, target_component) = state
        .vehicle_target
        .read()
        .await
        .ok_or("waiting for vehicle heartbeat")?;
    let items = mission_items(plan, target_system, target_component);
    let mut sent = vec![false; items.len()];
    let mut events = state.mission_events.subscribe();
    connection
        .send(
            &gcs_header(),
            &MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                target_system,
                target_component,
                count: items.len() as u16,
                mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
                ..Default::default()
            }),
        )
        .await
        .map_err(|e| format!("failed to announce mission: {e}"))?;
    let deadline = Instant::now() + MISSION_TIMEOUT;
    loop {
        let event = match timeout(
            deadline.saturating_duration_since(Instant::now()),
            events.recv(),
        )
        .await
        {
            Ok(Ok(event)) => event,
            Ok(Err(RecvError::Lagged(_))) => continue,
            _ => return Err("mission upload timed out waiting for ArduPilot".into()),
        };
        match event {
            MissionEvent::RequestInt(seq) | MissionEvent::Request(seq) => {
                let item = items
                    .get(seq as usize)
                    .ok_or_else(|| format!("vehicle requested invalid mission sequence {seq}"))?;
                // MAVLink specifies MISSION_ITEM_INT even for deprecated MISSION_REQUEST.
                let message = MavMessage::MISSION_ITEM_INT(item.clone());
                connection
                    .send(&gcs_header(), &message)
                    .await
                    .map_err(|e| format!("failed to send mission item {seq}: {e}"))?;
                sent[seq as usize] = true;
            }
            MissionEvent::Ack(result) => {
                if result != MavMissionResult::MAV_MISSION_ACCEPTED {
                    return Err(format!("ArduPilot rejected mission: {result:?}"));
                }
                if sent.iter().any(|sent| !sent) {
                    return Err(
                        "vehicle acknowledged before requesting the complete mission".into(),
                    );
                }
                return Ok(format!(
                    "ArduPilot accepted flight plan at {} m/s; arm and select AUTO to fly",
                    plan.settings.speed_m_s
                ));
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn plan(kind: MissionKind) -> MissionPlan {
        MissionPlan {
            home: HomePosition {
                lat: 24.7,
                lon: 46.6,
                alt_m: 600.0,
            },
            settings: MissionUploadRequest {
                kind,
                speed_m_s: 5.0,
                waypoints: if kind == MissionKind::Waypoints {
                    vec![MissionWaypoint {
                        lat: 24.71,
                        lon: 46.61,
                        alt_m: 25.0,
                    }]
                } else {
                    vec![]
                },
                orbit: if kind == MissionKind::Orbit {
                    Some(OrbitSettings {
                        diameter_m: 100.0,
                        alt_m: 30.0,
                    })
                } else {
                    None
                },
            },
        }
    }
    #[test]
    fn preserves_first_waypoint_and_encodes_speed_after_takeoff() {
        let p = plan(MissionKind::Waypoints);
        let items = mission_items(&p, 1, 1);
        assert_eq!(items.len(), 4);
        assert_eq!(items[0].frame, MavFrame::MAV_FRAME_GLOBAL);
        assert_eq!(items[0].z, 600.0);
        assert_eq!(items[1].command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert_eq!(items[2].command, MavCmd::MAV_CMD_DO_CHANGE_SPEED);
        assert_eq!(items[2].param1, 1.0);
        assert_eq!(items[2].param2, 5.0);
        assert_eq!(items[3].x, 247100000);
        assert_eq!(items[3].seq, 3);
    }
    #[test]
    fn orbit_centres_on_home_and_loops_without_repeating_takeoff() {
        let p = plan(MissionKind::Orbit);
        let items = mission_items(&p, 1, 1);
        assert_eq!(items[3].command, MavCmd::MAV_CMD_NAV_LOITER_TURNS);
        assert_eq!(items[3].x, 247000000);
        assert_eq!(items[3].y, 466000000);
        assert_eq!(items[3].param3, 50.0);
        assert_eq!(items[4].param1, 3.0);
        assert_eq!(items[4].param2, -1.0);
        assert!((orbit_rate(&p.settings) - 5.729578).abs() < 0.0001);
    }
    #[test]
    fn rejects_invalid_geometry_speed_and_mixed_plans() {
        let mut p = plan(MissionKind::Orbit).settings;
        assert!(validate_plan(&p).is_ok());
        p.orbit.as_mut().unwrap().diameter_m = 11.0;
        assert!(validate_plan(&p).is_err());
        p.orbit.as_mut().unwrap().diameter_m = 10.0;
        p.speed_m_s = 20.0;
        assert!(validate_plan(&p).is_err());
        p.speed_m_s = f32::NAN;
        assert!(validate_plan(&p).is_err());
        let mut p = plan(MissionKind::Waypoints).settings;
        p.waypoints[0].lat = 91.0;
        assert!(validate_plan(&p).is_err());
        p.waypoints.clear();
        assert!(validate_plan(&p).is_err());
    }
}
