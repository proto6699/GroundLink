//! Talking to the vehicle for Sweep: read battery parameters, command RTL.
//!
//! GroundLink only ever *reads* parameters here. It does not write the battery failsafe settings
//! for you; it tells you what the autopilot is set to so you can fix it on purpose.

use crate::{
    mission::{MissionEvent, gcs_header},
    state::AppState,
    telemetry::DataSource,
};
use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, PARAM_REQUEST_READ_DATA,
};
use serde::Serialize;
use std::sync::atomic::Ordering;
use tokio::{
    sync::broadcast::error::RecvError,
    time::{Duration, Instant, sleep, timeout},
};

/// ArduCopter custom mode number for RTL.
const COPTER_MODE_RTL: f32 = 6.0;

#[derive(Debug, Clone, Serialize)]
pub struct Failsafe {
    pub monitor: Option<f32>,
    pub low_mah: Option<f32>,
    pub crt_mah: Option<f32>,
    pub low_act: Option<f32>,
    pub crt_act: Option<f32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct VehicleInfo {
    pub source: &'static str,
    pub connected: bool,
    pub armed: bool,
    pub battery_pct: Option<u8>,
    pub voltage_mv: Option<u32>,
    pub current_a: Option<f32>,
    /// Only the demo has a figure to offer before the aircraft has ever flown.
    pub typical_current_a: Option<f64>,
    pub capacity_mah: Option<f64>,
    pub capacity_source: Option<&'static str>,
    pub failsafe: Option<Failsafe>,
    pub notes: Vec<String>,
}

/// Read what the pre-flight form can fill in by itself.
pub async fn scan(state: &AppState) -> VehicleInfo {
    let source = *state.source.read().await;
    let connected = *state.connected.read().await;
    let latest = state.latest.read().await.clone();
    let mut info = VehicleInfo {
        source: match source {
            DataSource::Demo => "demo",
            DataSource::Sitl => "sitl",
        },
        connected,
        armed: latest.as_ref().is_some_and(|t| t.armed),
        battery_pct: latest.as_ref().and_then(|t| t.battery_pct),
        voltage_mv: latest.as_ref().and_then(|t| t.voltage_mv),
        current_a: latest.as_ref().and_then(|t| t.current_a),
        typical_current_a: None,
        capacity_mah: None,
        capacity_source: None,
        failsafe: None,
        notes: Vec::new(),
    };

    match source {
        DataSource::Demo => {
            info.capacity_mah = Some(crate::demo::DEMO_CAPACITY_MAH);
            info.capacity_source = Some("demo");
            info.typical_current_a = Some((f64::from(crate::demo::DEMO_CURRENT_A) * 10.0).round() / 10.0);
            info.notes
                .push("demo mode: battery capacity and current are simulated, not read from a vehicle".into());
        }
        DataSource::Sitl if !connected => {
            info.notes
                .push("vehicle not connected: capacity is unknown, enter it by hand".into());
        }
        DataSource::Sitl => {
            const NAMES: [&str; 6] = [
                "BATT_CAPACITY",
                "BATT_MONITOR",
                "BATT_LOW_MAH",
                "BATT_CRT_MAH",
                "BATT_FS_LOW_ACT",
                "BATT_FS_CRT_ACT",
            ];
            match read_params(state, &NAMES).await {
                Ok(v) => {
                    match v[0] {
                        Some(cap) if cap > 0.0 => {
                            info.capacity_mah = Some(cap as f64);
                            info.capacity_source = Some("vehicle");
                        }
                        Some(_) => info.notes.push(
                            "BATT_CAPACITY is 0 on the vehicle: enter the pack capacity by hand".into(),
                        ),
                        None => info.notes.push(
                            "could not read BATT_CAPACITY from the vehicle: enter it by hand".into(),
                        ),
                    }
                    if v[1] == Some(0.0) {
                        info.notes.push(
                            "BATT_MONITOR is 0: the autopilot is not measuring the battery, so percent and current are not trustworthy".into(),
                        );
                    }
                    if v[4] == Some(0.0) {
                        info.notes.push(
                            "BATT_FS_LOW_ACT is 0: the autopilot will do nothing on low battery. Set it to RTL; that is the real safety net".into(),
                        );
                    }
                    if v[5] == Some(0.0) {
                        info.notes.push(
                            "BATT_FS_CRT_ACT is 0: the autopilot will do nothing on critical battery".into(),
                        );
                    }
                    if v[2].is_some_and(|x| x <= 0.0) && v[3].is_some_and(|x| x <= 0.0) {
                        info.notes.push(
                            "BATT_LOW_MAH and BATT_CRT_MAH are both 0: the failsafe only triggers on voltage".into(),
                        );
                    }
                    info.failsafe = Some(Failsafe {
                        monitor: v[1],
                        low_mah: v[2],
                        crt_mah: v[3],
                        low_act: v[4],
                        crt_act: v[5],
                    });
                }
                Err(error) => info.notes.push(format!("parameter read failed: {error}")),
            }
        }
    }
    info
}

/// Read named parameters. Missing ones come back as `None` after a few retries.
pub async fn read_params(state: &AppState, names: &[&str]) -> Result<Vec<Option<f32>>, String> {
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
    let mut values: Vec<Option<f32>> = vec![None; names.len()];

    for _ in 0..3 {
        for (index, name) in names.iter().enumerate() {
            if values[index].is_some() {
                continue;
            }
            let request = MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                target_system,
                target_component,
                param_id: (*name).into(),
                param_index: -1,
            });
            connection
                .send(&gcs_header(), &request)
                .await
                .map_err(|e| format!("parameter request failed: {e}"))?;
        }
        let deadline = Instant::now() + Duration::from_millis(1500);
        while values.iter().any(Option::is_none) {
            match timeout(
                deadline.saturating_duration_since(Instant::now()),
                events.recv(),
            )
            .await
            {
                Ok(Ok(MissionEvent::Parameter(name, value))) => {
                    if let Some(index) = names.iter().position(|n| *n == name)
                        && value.is_finite()
                    {
                        values[index] = Some(value);
                    }
                }
                Ok(Ok(_)) | Ok(Err(RecvError::Lagged(_))) => continue,
                _ => break,
            }
        }
        if values.iter().all(Option::is_some) {
            break;
        }
    }
    Ok(values)
}

/// Switch the vehicle to RTL and wait to see it happen.
///
/// The simulated demo vehicle has no flight controller, so there the request is a flag the demo
/// loop reads.
pub async fn command_rtl(state: &AppState) -> Result<(), String> {
    if matches!(*state.source.read().await, DataSource::Demo) {
        state.sim_rtl.store(true, Ordering::SeqCst);
        return Ok(());
    }
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
    for _ in 0..3 {
        let message = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system,
            target_component,
            command: MavCmd::MAV_CMD_DO_SET_MODE,
            // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, then the ArduCopter mode number.
            param1: 1.0,
            param2: COPTER_MODE_RTL,
            ..Default::default()
        });
        connection
            .send(&gcs_header(), &message)
            .await
            .map_err(|e| format!("could not send RTL: {e}"))?;
        let deadline = Instant::now() + Duration::from_millis(2500);
        while Instant::now() < deadline {
            if state
                .latest
                .read()
                .await
                .as_ref()
                .is_some_and(|t| t.flight_mode == "RTL")
            {
                return Ok(());
            }
            sleep(Duration::from_millis(200)).await;
        }
    }
    Err("the vehicle never reported RTL".into())
}
