use crate::{
    mission::MissionWaypoint,
    state::AppState,
    telemetry::{DataSource, ServerMessage, Telemetry, now_ms},
};
use std::{f64::consts::TAU, time::Duration};
use tokio::time::{Instant, interval};
use tracing::info;

const BASE_LAT: f64 = 24.7136;
const BASE_LON: f64 = 46.6753;
const LOOP_SECS: f64 = 120.0;
const LEG_SECS: f64 = 9.0;

pub async fn run(state: AppState) {
    info!("native demo telemetry source online");

    *state.connected.write().await = true;
    *state.source.write().await = DataSource::Demo;
    let _ = state.tx.send(ServerMessage::Status {
        connected: true,
        source: DataSource::Demo,
    });

    let started = Instant::now();
    let mut ticker = interval(Duration::from_millis(250));

    loop {
        ticker.tick().await;

        let elapsed = started.elapsed().as_secs_f64();
        let mission = state.mission.read().await.clone();
        let telemetry = if mission.is_empty() {
            default_demo_telemetry(elapsed)
        } else {
            mission_demo_telemetry(elapsed, &mission)
        };

        *state.latest.write().await = Some(telemetry.clone());
        let _ = state.tx.send(ServerMessage::Telemetry { data: telemetry });
    }
}

fn default_demo_telemetry(elapsed: f64) -> Telemetry {
    let cycle = elapsed % LOOP_SECS;
    let armed = (5.0..108.0).contains(&cycle);
    let flight_mode = mode_for_cycle(cycle).to_string();

    let flight_progress = ((cycle - 5.0).max(0.0) / 103.0).clamp(0.0, 1.0);
    let angle = flight_progress * TAU * 1.7;
    let radius = if armed { 0.0012 } else { 0.0 };

    let lat = BASE_LAT + radius * angle.sin();
    let lon = BASE_LON + radius * angle.cos();
    let altitude = demo_altitude(cycle, armed);
    let battery = (100.0 - flight_progress * 82.0).round().clamp(18.0, 100.0) as u8;
    let voltage_mv = 16_800_u32.saturating_sub((flight_progress * 2_600.0) as u32);
    let gps_sats = if (78.0..88.0).contains(&cycle) { 5 } else { 14 };

    Telemetry {
        timestamp_ms: now_ms(),
        lat,
        lon,
        alt_m: altitude as f32,
        battery_pct: Some(battery),
        voltage_mv: Some(voltage_mv),
        gps_sats: Some(gps_sats),
        armed,
        flight_mode,
        roll_deg: (elapsed * 0.72).sin() as f32 * 17.0,
        pitch_deg: (elapsed * 0.39).sin() as f32 * 7.0,
        yaw_deg: ((angle.to_degrees() + 90.0) % 360.0) as f32,
    }
}

fn mission_demo_telemetry(elapsed: f64, mission: &[MissionWaypoint]) -> Telemetry {
    let count = mission.len();
    let total = (count.max(1) as f64) * LEG_SECS;
    let phase = elapsed % total;
    let index = ((phase / LEG_SECS).floor() as usize).min(count - 1);
    let next_index = (index + 1) % count;
    let local_t = (phase % LEG_SECS) / LEG_SECS;
    let eased = local_t * local_t * (3.0 - 2.0 * local_t);

    let a = &mission[index];
    let b = &mission[next_index];
    let lat = lerp(a.lat, b.lat, eased);
    let lon = lerp(a.lon, b.lon, eased);
    let alt = lerp(a.alt_m as f64, b.alt_m as f64, eased) as f32;

    let north = (b.lat - a.lat) * 111_320.0;
    let east = (b.lon - a.lon) * 111_320.0 * a.lat.to_radians().cos();
    let yaw = east.atan2(north).to_degrees().rem_euclid(360.0) as f32;
    let turn = ((local_t - 0.5) * std::f64::consts::PI).sin() as f32;

    Telemetry {
        timestamp_ms: now_ms(),
        lat,
        lon,
        alt_m: alt,
        battery_pct: Some((96.0 - (elapsed % 240.0) * 0.22).clamp(35.0, 96.0) as u8),
        voltage_mv: Some((16_650.0 - (elapsed % 240.0) * 5.0).clamp(14_300.0, 16_650.0) as u32),
        gps_sats: Some(15),
        armed: true,
        flight_mode: "AUTO".to_string(),
        roll_deg: turn * 22.0,
        pitch_deg: ((elapsed * 0.7).sin() * 4.5) as f32,
        yaw_deg: yaw,
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn mode_for_cycle(cycle: f64) -> &'static str {
    match cycle {
        t if t < 5.0 => "STABILIZE",
        t if t < 25.0 => "LOITER",
        t if t < 65.0 => "AUTO",
        t if t < 92.0 => "GUIDED",
        t if t < 104.0 => "RTL",
        t if t < 108.0 => "LAND",
        _ => "STABILIZE",
    }
}

fn demo_altitude(cycle: f64, armed: bool) -> f64 {
    if !armed {
        return 0.0;
    }

    if cycle < 12.0 {
        return ((cycle - 5.0) / 7.0 * 28.0).clamp(0.0, 28.0);
    }

    if cycle > 100.0 {
        return ((108.0 - cycle) / 8.0 * 28.0).clamp(0.0, 28.0);
    }

    28.0 + ((cycle - 12.0) / 4.5).sin() * 8.0
}
