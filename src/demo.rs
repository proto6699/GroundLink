use crate::{
    mission::{HomePosition, MissionKind, MissionPlan},
    state::AppState,
    telemetry::{DataSource, ServerMessage, Telemetry, now_ms},
};
use std::{f64::consts::TAU, time::Duration};
use tokio::time::{Instant, interval};
use tracing::info;

const BASE_LAT: f64 = 24.7136;
const BASE_LON: f64 = 46.6753;
const LOOP_SECS: f64 = 120.0;

pub async fn run(state: AppState) {
    info!("native demo telemetry source online");

    let home = HomePosition {
        lat: BASE_LAT,
        lon: BASE_LON,
        alt_m: 600.0,
    };
    *state.home.write().await = Some(home.clone());
    let _ = state.tx.send(ServerMessage::Home { home: Some(home) });
    *state.connected.write().await = true;
    *state.source.write().await = DataSource::Demo;
    let _ = state.tx.send(ServerMessage::Status {
        connected: true,
        source: DataSource::Demo,
    });

    let started = Instant::now();
    let mut ticker = interval(Duration::from_millis(250));

    let mut active_plan = String::new();
    let mut plan_started = Instant::now();
    loop {
        ticker.tick().await;

        let elapsed = started.elapsed().as_secs_f64();
        let mission = state.mission.read().await.clone();
        let signature = serde_json::to_string(&mission).unwrap_or_default();
        if signature != active_plan {
            active_plan = signature;
            plan_started = Instant::now();
        }
        let telemetry = match mission {
            Some(plan) => mission_demo_telemetry(plan_started.elapsed().as_secs_f64(), &plan),
            None => default_demo_telemetry(elapsed),
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

fn mission_demo_telemetry(elapsed: f64, plan: &MissionPlan) -> Telemetry {
    let speed = plan.settings.speed_m_s as f64;
    let (lat, lon, alt_m, yaw_deg, roll_deg) = if plan.settings.kind == MissionKind::Orbit {
        let orbit = plan.settings.orbit.as_ref().unwrap();
        let radius = orbit.diameter_m as f64 / 2.0;
        let angle = elapsed * speed / radius;
        (
            plan.home.lat + radius * angle.cos() / 111_320.0,
            plan.home.lon + radius * angle.sin() / (111_320.0 * plan.home.lat.to_radians().cos()),
            orbit.alt_m,
            (angle.to_degrees() + 180.0).rem_euclid(360.0) as f32,
            (speed * speed / (9.81 * radius)).atan().to_degrees() as f32,
        )
    } else {
        // Start at home, fly each leg at the selected horizontal speed, hold the last waypoint.
        let mut a = crate::mission::MissionWaypoint {
            lat: plan.home.lat,
            lon: plan.home.lon,
            alt_m: plan.settings.waypoints[0].alt_m,
        };
        let mut remaining = elapsed;
        let mut position = (a.lat, a.lon, a.alt_m, 0.0, 0.0);
        for b in &plan.settings.waypoints {
            let north = (b.lat - a.lat) * 111_320.0;
            let east = (b.lon - a.lon) * 111_320.0 * a.lat.to_radians().cos();
            let duration = north.hypot(east) / speed;
            let yaw = east.atan2(north).to_degrees().rem_euclid(360.0) as f32;
            if remaining < duration {
                let t = remaining / duration;
                position = (
                    lerp(a.lat, b.lat, t),
                    lerp(a.lon, b.lon, t),
                    lerp(a.alt_m as f64, b.alt_m as f64, t) as f32,
                    yaw,
                    0.0,
                );
                break;
            }
            remaining -= duration;
            position = (b.lat, b.lon, b.alt_m, yaw, 0.0);
            a = b.clone();
        }
        position
    };
    Telemetry {
        timestamp_ms: now_ms(),
        lat,
        lon,
        alt_m,
        yaw_deg,
        roll_deg,
        battery_pct: Some((96.0 - (elapsed % 240.0) * 0.22).clamp(35.0, 96.0) as u8),
        voltage_mv: Some((16_650.0 - (elapsed % 240.0) * 5.0).clamp(14_300.0, 16_650.0) as u32),
        gps_sats: Some(15),
        armed: true,
        flight_mode: if plan.settings.kind == MissionKind::Orbit {
            "CIRCLE / AUTO".into()
        } else {
            "AUTO".into()
        },
        pitch_deg: ((elapsed * 0.7).sin() * 4.5) as f32,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::{MissionUploadRequest, MissionWaypoint, OrbitSettings};
    #[test]
    fn orbit_geometry_and_speed_use_home() {
        let mut plan = MissionPlan {
            home: HomePosition {
                lat: BASE_LAT,
                lon: BASE_LON,
                alt_m: 600.0,
            },
            settings: MissionUploadRequest {
                kind: MissionKind::Orbit,
                waypoints: vec![],
                speed_m_s: 5.0,
                orbit: Some(OrbitSettings {
                    diameter_m: 100.0,
                    alt_m: 30.0,
                }),
            },
        };
        let t = mission_demo_telemetry(0.0, &plan);
        assert!(((t.lat - BASE_LAT) * 111_320.0 - 50.0).abs() < 0.001);
        let quarter = std::f64::consts::FRAC_PI_2 * 50.0 / 5.0;
        let a = mission_demo_telemetry(quarter, &plan);
        assert!((a.lat - BASE_LAT).abs() < 1e-7);
        plan.settings.speed_m_s = 10.0;
        let b = mission_demo_telemetry(quarter / 2.0, &plan);
        assert!((a.lon - b.lon).abs() < 1e-7);
    }
    #[test]
    fn waypoint_demo_holds_final_destination() {
        let plan = MissionPlan {
            home: HomePosition {
                lat: BASE_LAT,
                lon: BASE_LON,
                alt_m: 600.0,
            },
            settings: MissionUploadRequest {
                kind: MissionKind::Waypoints,
                speed_m_s: 5.0,
                orbit: None,
                waypoints: vec![MissionWaypoint {
                    lat: BASE_LAT + 0.001,
                    lon: BASE_LON,
                    alt_m: 25.0,
                }],
            },
        };
        let t = mission_demo_telemetry(1000.0, &plan);
        assert_eq!(t.lat, BASE_LAT + 0.001);
    }
}
