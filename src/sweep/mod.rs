//! Sweep: a survey-area ("roomba") planner.
//!
//! Draw an area, get lanes. Before anything is uploaded the pre-flight check works out whether one
//! battery can cover the area and fly home, and while the sweep is flying the monitor keeps asking
//! the same question. It lives in its own module so the rest of GroundLink does not depend on it;
//! set `GROUNDLINK_SWEEP=0` to switch the whole thing off.

pub mod estimate;
pub mod geo;
pub mod monitor;
pub mod plan;
mod upload;
mod vehicle;

use crate::{
    mission::HomePosition,
    state::AppState,
    telemetry::{DataSource, ServerMessage},
};
use axum::{
    Json, Router,
    extract::State,
    http::StatusCode,
    routing::{get, post},
};
use estimate::{BatteryInput, Estimate, Route, Verdict, estimate};
use geo::{LatLon, distance_m};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::sync::atomic::{AtomicU64, Ordering};

pub use monitor::SweepProgress;

static NEXT_PLAN_ID: AtomicU64 = AtomicU64::new(1);

/// `GROUNDLINK_SWEEP=0` (or `false` / `off`) turns Sweep off. On by default.
pub fn enabled_from_env() -> bool {
    !matches!(
        std::env::var("GROUNDLINK_SWEEP")
            .unwrap_or_default()
            .trim()
            .to_ascii_lowercase()
            .as_str(),
        "0" | "false" | "off" | "no"
    )
}

/// A sweep that has been uploaded and is being watched.
#[derive(Debug, Clone, Serialize)]
pub struct SweepPlan {
    pub id: u64,
    /// Lane endpoints in flight order: lane `i` runs `path[2i] -> path[2i + 1]`.
    pub path: Vec<LatLon>,
    pub lane_count: usize,
    pub angle_deg: f64,
    pub spacing_m: f64,
    pub effective_spacing_m: f64,
    pub alt_m: f32,
    pub speed_m_s: f32,
    pub home: HomePosition,
    pub path_m: f64,
    pub out_m: f64,
    pub back_m: f64,
    pub battery: BatteryInput,
    pub auto_rtl: bool,
    pub estimate: Estimate,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Deserialize)]
pub struct SweepRequest {
    pub polygon: Vec<LatLon>,
    pub spacing_m: f64,
    /// Compass bearing the lanes run along. Omit to let Sweep pick the direction with fewest turns.
    #[serde(default)]
    pub angle_deg: Option<f64>,
    pub alt_m: f32,
    pub speed_m_s: f32,
    /// Needed to upload. Optional for a preview, which then shows the lanes without a verdict.
    #[serde(default)]
    pub battery: Option<BatteryInput>,
    /// GroundLink commands RTL itself if the battery cannot finish the sweep.
    #[serde(default = "yes")]
    pub auto_rtl: bool,
}

fn validate_flight(req: &SweepRequest) -> Result<(), String> {
    if !req.alt_m.is_finite() || !(1.0..=500.0).contains(&req.alt_m) {
        return Err("altitude must be 1–500 m above home".into());
    }
    if !req.speed_m_s.is_finite() || !(0.5..=20.0).contains(&req.speed_m_s) {
        return Err("speed must be 0.5–20 m/s".into());
    }
    Ok(())
}

/// Lanes plus the distances needed to budget the flight. Without a known home the transit legs are
/// left out of the budget.
fn prepare(
    req: &SweepRequest,
    home: Option<&HomePosition>,
) -> Result<(plan::LanePlan, Route), String> {
    validate_flight(req)?;
    let lanes = plan::plan_lanes(&req.polygon, req.spacing_m, req.angle_deg)?;
    let (out_m, back_m) = match (home, lanes.path.first(), lanes.path.last()) {
        (Some(home), Some(&first), Some(&last)) => {
            let home = LatLon {
                lat: home.lat,
                lon: home.lon,
            };
            (distance_m(home, first), distance_m(last, home))
        }
        _ => (0.0, 0.0),
    };
    let route = Route {
        sweep_m: lanes.path_m,
        turns: lanes.turns,
        out_m,
        back_m,
        alt_m: req.alt_m as f64,
        speed_m_s: req.speed_m_s as f64,
    };
    Ok((lanes, route))
}

pub fn routes(enabled: bool) -> Router<AppState> {
    let router = Router::new().route("/api/sweep/config", get(config_handler));
    if !enabled {
        return router;
    }
    router
        .route("/api/sweep/preview", post(preview_handler))
        .route("/api/sweep/vehicle", get(vehicle_handler))
        .route("/api/sweep/upload", post(upload_handler))
        .route("/api/sweep/clear", post(clear_handler))
}

async fn config_handler(State(state): State<AppState>) -> Json<Value> {
    Json(json!({
        "enabled": state.sweep_enabled,
        "max_lanes": plan::MAX_PATH_POINTS / 2,
        "min_spacing_m": plan::MIN_SPACING_M,
        "max_spacing_m": plan::MAX_SPACING_M,
        "default_reserve_pct": estimate::DEFAULT_RESERVE_PCT,
    }))
}

async fn preview_handler(
    State(state): State<AppState>,
    Json(req): Json<SweepRequest>,
) -> (StatusCode, Json<Value>) {
    let home = state.home.read().await.clone();
    let (lanes, route) = match prepare(&req, home.as_ref()) {
        Ok(prepared) => prepared,
        Err(error) => {
            return (
                StatusCode::BAD_REQUEST,
                Json(json!({"ok":false,"error":error})),
            );
        }
    };
    let (budget, battery_error) = match &req.battery {
        None => (None, None),
        Some(battery) => match battery.validate() {
            Ok(()) => (Some(estimate(&route, battery)), None),
            Err(error) => (None, Some(error)),
        },
    };
    (
        StatusCode::OK,
        Json(json!({
            "ok": true,
            "lanes": lanes,
            "estimate": budget,
            "battery_error": battery_error,
            "home_known": home.is_some(),
            "out_m": route.out_m,
            "back_m": route.back_m,
        })),
    )
}

async fn vehicle_handler(State(state): State<AppState>) -> Json<vehicle::VehicleInfo> {
    Json(vehicle::scan(&state).await)
}

async fn upload_handler(
    State(state): State<AppState>,
    Json(req): Json<SweepRequest>,
) -> (StatusCode, Json<Value>) {
    let bad = |status: StatusCode, error: String| (status, Json(json!({"ok":false,"error":error})));
    let Some(home) = state.home.read().await.clone() else {
        return bad(
            StatusCode::CONFLICT,
            "waiting for vehicle HOME_POSITION; home is never inferred from the moving drone".into(),
        );
    };
    let (lanes, route) = match prepare(&req, Some(&home)) {
        Ok(prepared) => prepared,
        Err(error) => return bad(StatusCode::BAD_REQUEST, error),
    };
    let Some(battery) = req.battery else {
        return bad(
            StatusCode::BAD_REQUEST,
            "pre-flight needs the battery capacity, charge left and average current".into(),
        );
    };
    if let Err(error) = battery.validate() {
        return bad(StatusCode::BAD_REQUEST, error);
    }
    // The server re-checks the budget itself; the browser's verdict is not trusted.
    let budget = estimate(&route, &battery);
    if budget.verdict == Verdict::TooLong {
        return (
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({
                "ok": false,
                "error": format!("too long for this battery: {}", budget.describe()),
                "estimate": budget,
            })),
        );
    }
    let Ok(_upload) = state.mission_upload.try_lock() else {
        return bad(
            StatusCode::CONFLICT,
            "another mission upload is in progress".into(),
        );
    };

    let plan = SweepPlan {
        id: NEXT_PLAN_ID.fetch_add(1, Ordering::SeqCst),
        lane_count: lanes.lane_count,
        angle_deg: lanes.angle_deg,
        spacing_m: lanes.spacing_m,
        effective_spacing_m: lanes.effective_spacing_m,
        path: lanes.path,
        alt_m: req.alt_m,
        speed_m_s: req.speed_m_s,
        home,
        path_m: route.sweep_m,
        out_m: route.out_m,
        back_m: route.back_m,
        battery,
        auto_rtl: req.auto_rtl,
        estimate: budget,
    };
    let source = *state.source.read().await;
    let result = match source {
        DataSource::Demo => Ok("demo sweep loaded".to_string()),
        DataSource::Sitl => upload::upload_to_vehicle(&state, &plan).await,
    };
    match result {
        Ok(message) => {
            install(&state, plan.clone()).await;
            (
                StatusCode::OK,
                Json(json!({"ok":true,"message":message,"plan":plan})),
            )
        }
        Err(error) => bad(StatusCode::BAD_GATEWAY, error),
    }
}

/// Make `plan` the sweep GroundLink is watching. The vehicle holds one mission at a time, so the
/// ordinary waypoint mission GroundLink remembers is dropped.
async fn install(state: &AppState, plan: SweepPlan) {
    state.sim_rtl.store(false, Ordering::SeqCst);
    *state.sweep_progress.write().await = None;
    *state.mission.write().await = None;
    *state.sweep.write().await = Some(plan.clone());
    let _ = state.tx.send(ServerMessage::Sweep { plan: Some(plan) });
}

/// Stop watching the loaded sweep. The vehicle's mission is untouched until the next upload.
pub async fn clear(state: &AppState) {
    state.sim_rtl.store(false, Ordering::SeqCst);
    *state.sweep_progress.write().await = None;
    if state.sweep.write().await.take().is_some() {
        let _ = state.tx.send(ServerMessage::Sweep { plan: None });
    }
}

async fn clear_handler(State(state): State<AppState>) -> Json<Value> {
    clear(&state).await;
    Json(json!({"ok":true}))
}

/// A small ready-made plan for tests in other modules.
#[cfg(test)]
pub(crate) fn sample_plan() -> SweepPlan {
    let route = Route {
        sweep_m: 100.0,
        turns: 1,
        out_m: 10.0,
        back_m: 10.0,
        alt_m: 30.0,
        speed_m_s: 5.0,
    };
    let battery = BatteryInput {
        capacity_mah: 5000.0,
        remaining_pct: 100.0,
        avg_current_a: 20.0,
        reserve_pct: 30.0,
    };
    SweepPlan {
        id: 1,
        path: vec![
            LatLon { lat: 24.7, lon: 46.6 },
            LatLon { lat: 24.701, lon: 46.6 },
            LatLon { lat: 24.701, lon: 46.6001 },
            LatLon { lat: 24.7, lon: 46.6001 },
        ],
        lane_count: 2,
        angle_deg: 0.0,
        spacing_m: 10.0,
        effective_spacing_m: 10.0,
        alt_m: 30.0,
        speed_m_s: 5.0,
        home: HomePosition {
            lat: 24.6999,
            lon: 46.5999,
            alt_m: 600.0,
        },
        path_m: 100.0,
        out_m: 10.0,
        back_m: 10.0,
        battery,
        auto_rtl: true,
        estimate: estimate(&route, &battery),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(battery: Option<BatteryInput>) -> SweepRequest {
        let frame = geo::Frame::new(LatLon { lat: 26.4, lon: 50.1 });
        SweepRequest {
            polygon: [(0.0, 0.0), (120.0, 0.0), (120.0, 80.0), (0.0, 80.0)]
                .iter()
                .map(|&p| frame.to_latlon(p))
                .collect(),
            spacing_m: 20.0,
            angle_deg: Some(90.0),
            alt_m: 30.0,
            speed_m_s: 5.0,
            battery,
            auto_rtl: true,
        }
    }

    #[test]
    fn transit_is_counted_only_when_home_is_known() {
        let req = request(None);
        let (_, without) = prepare(&req, None).unwrap();
        assert_eq!((without.out_m, without.back_m), (0.0, 0.0));
        let frame = geo::Frame::new(LatLon { lat: 26.4, lon: 50.1 });
        let home_at = frame.to_latlon((0.0, -100.0));
        let home = HomePosition {
            lat: home_at.lat,
            lon: home_at.lon,
            alt_m: 10.0,
        };
        let (lanes, with) = prepare(&req, Some(&home)).unwrap();
        assert!(with.out_m > 100.0 && with.back_m > 0.0);
        assert_eq!(with.sweep_m, lanes.path_m);
    }

    #[test]
    fn flight_settings_are_validated() {
        let mut req = request(None);
        req.alt_m = 0.0;
        assert!(prepare(&req, None).is_err());
        let mut req = request(None);
        req.speed_m_s = 25.0;
        assert!(prepare(&req, None).is_err());
        let mut req = request(None);
        req.speed_m_s = f32::NAN;
        assert!(prepare(&req, None).is_err());
    }

    #[test]
    fn request_defaults_match_the_ui_contract() {
        let json = r#"{"polygon":[{"lat":26.4,"lon":50.1},{"lat":26.4,"lon":50.101},{"lat":26.401,"lon":50.101}],
            "spacing_m":15,"alt_m":30,"speed_m_s":5}"#;
        let req: SweepRequest = serde_json::from_str(json).unwrap();
        assert!(req.auto_rtl, "auto-RTL is on unless the pilot turns it off");
        assert!(req.angle_deg.is_none() && req.battery.is_none());
    }

    #[test]
    fn env_flag_parsing() {
        // Default (unset) is on; the explicit off spellings are the only things that disable it.
        // Mutating the environment is racy across tests, so only the pure default is asserted.
        if std::env::var("GROUNDLINK_SWEEP").is_err() {
            assert!(enabled_from_env());
        }
    }
}
