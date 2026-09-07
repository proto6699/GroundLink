use crate::mission::MissionWaypoint;
use mavlink::dialects::ardupilotmega::{MavModeFlag, MavMessage};
use serde::Serialize;
use std::{f32::consts::PI, time::{SystemTime, UNIX_EPOCH}};

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum DataSource {
    Sitl,
    Demo,
}

#[derive(Debug, Clone, Serialize)]
pub struct Telemetry {
    pub timestamp_ms: u64,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f32,
    pub battery_pct: Option<u8>,
    pub voltage_mv: Option<u32>,
    pub gps_sats: Option<u8>,
    pub armed: bool,
    pub flight_mode: String,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            timestamp_ms: now_ms(),
            lat: 0.0,
            lon: 0.0,
            alt_m: 0.0,
            battery_pct: None,
            voltage_mv: None,
            gps_sats: None,
            armed: false,
            flight_mode: "UNKNOWN".to_string(),
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    Telemetry { data: Telemetry },
    Status { connected: bool, source: DataSource },
    Mission { waypoints: Vec<MissionWaypoint> },
}

impl Telemetry {
    pub fn apply_mavlink(&mut self, message: &MavMessage) -> bool {
        let mut changed = false;

        match message {
            MavMessage::GLOBAL_POSITION_INT(data) => {
                self.lat = data.lat as f64 / 10_000_000.0;
                self.lon = data.lon as f64 / 10_000_000.0;
                self.alt_m = data.relative_alt as f32 / 1_000.0;
                changed = true;
            }
            MavMessage::SYS_STATUS(data) => {
                self.voltage_mv = (data.voltage_battery != u16::MAX)
                    .then_some(data.voltage_battery as u32);
                self.battery_pct = (data.battery_remaining >= 0)
                    .then_some(data.battery_remaining as u8);
                changed = true;
            }
            MavMessage::GPS_RAW_INT(data) => {
                self.gps_sats = (data.satellites_visible != u8::MAX)
                    .then_some(data.satellites_visible);
                changed = true;
            }
            MavMessage::HEARTBEAT(data) => {
                self.armed = data
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                self.flight_mode = arducopter_mode_name(data.custom_mode).to_string();
                changed = true;
            }
            MavMessage::ATTITUDE(data) => {
                self.roll_deg = radians_to_degrees(data.roll);
                self.pitch_deg = radians_to_degrees(data.pitch);
                self.yaw_deg = normalize_heading(radians_to_degrees(data.yaw));
                changed = true;
            }
            _ => {}
        }

        if changed {
            self.timestamp_ms = now_ms();
        }

        changed
    }
}

pub fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn radians_to_degrees(value: f32) -> f32 {
    value * 180.0 / PI
}

fn normalize_heading(value: f32) -> f32 {
    value.rem_euclid(360.0)
}

fn arducopter_mode_name(mode: u32) -> &'static str {
    match mode {
        0 => "STABILIZE",
        1 => "ACRO",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        9 => "LAND",
        11 => "DRIFT",
        13 => "SPORT",
        14 => "FLIP",
        15 => "AUTOTUNE",
        16 => "POSHOLD",
        17 => "BRAKE",
        18 => "THROW",
        19 => "AVOID_ADSB",
        20 => "GUIDED_NOGPS",
        21 => "SMART_RTL",
        22 => "FLOWHOLD",
        23 => "FOLLOW",
        24 => "ZIGZAG",
        25 => "SYSTEMID",
        26 => "AUTOROTATE",
        27 => "AUTO_RTL",
        28 => "TURTLE",
        _ => "UNKNOWN",
    }
}
