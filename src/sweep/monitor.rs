//! The in-flight half of Sweep.
//!
//! While a sweep is loaded and the vehicle is flying it in AUTO, this tracks how far along the
//! lanes the aircraft is (so the map can show them being eaten) and keeps asking one question:
//! can the battery still finish the job and bring it home? If not, it commands RTL once.
//!
//! This is a second line of defence. The autopilot's own battery failsafe (`BATT_FS_*`) is the real
//! safety net because it keeps working when GroundLink or the radio link does not.

use super::{
    SweepPlan,
    estimate::{Remaining, must_return, remaining_need_mah, usable_now_mah},
    geo::{Frame, LatLon, Path},
    vehicle,
};
use crate::{
    state::AppState,
    telemetry::{ServerMessage, now_ms},
};
use serde::Serialize;
use tokio::time::{Duration, Instant, MissedTickBehavior, interval};
use tracing::{info, warn};

const TICK: Duration = Duration::from_millis(250);
const STALE_MS: u64 = 3000;
/// Within this many metres of the end of the last lane counts as finished.
const DONE_M: f64 = 1.5;
/// Consecutive ticks (about a second) the answer must be "turn back" before acting on it.
const STRIKES_NEEDED: u8 = 4;
/// Ignore the first couple of seconds of AUTO: takeoff and acceleration are not representative.
const SETTLE_TICKS: u32 = 8;
/// Only start averaging live current after this many AUTO ticks (about ten seconds).
const AMPS_EMA_AFTER_TICKS: u32 = 40;
const AMPS_EMA_ALPHA: f64 = 0.02;
const REPORT_EVERY: Duration = Duration::from_secs(1);
const REPORT_STEP_M: f64 = 0.5;

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    /// Loaded, but the aircraft has not reached the first lane yet.
    Transit,
    Sweeping,
    Done,
    /// Interrupted: heading home before the lanes were finished.
    Rtl,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct SweepProgress {
    pub plan_id: u64,
    pub phase: Phase,
    /// Metres eaten along the route.
    pub s_m: f64,
    pub total_m: f64,
    /// Lanes completely eaten.
    pub lane: usize,
    pub lanes: usize,
    /// Latest in-flight estimate: energy to finish and come home, and charge above the reserve.
    pub need_mah: Option<f64>,
    pub usable_mah: Option<f64>,
    pub rtl_commanded: bool,
}

struct BatteryCheck {
    need_mah: f64,
    usable_mah: f64,
    trigger: bool,
}

struct Tracker {
    plan_id: u64,
    frame: Frame,
    path: Path,
    home_xy: (f64, f64),
    lanes: usize,
    alt_m: f64,
    speed_m_s: f64,
    spacing_m: f64,
    plan_amps: f64,
    capacity_mah: f64,
    reserve_pct: f64,
    s: f64,
    started: bool,
    auto_ticks: u32,
    amps_ema: Option<f64>,
    strikes: u8,
    rtl_commanded: bool,
    noted_start: bool,
    noted_done: bool,
    last_need: Option<f64>,
    last_usable: Option<f64>,
    last_report: Option<(Instant, SweepProgress)>,
}

impl Tracker {
    fn new(plan: &SweepPlan) -> Self {
        let frame = Frame::new(plan.path[0]);
        let path = Path::new(plan.path.iter().map(|&p| frame.to_xy(p)).collect());
        let home_xy = frame.to_xy(LatLon {
            lat: plan.home.lat,
            lon: plan.home.lon,
        });
        Self {
            plan_id: plan.id,
            frame,
            path,
            home_xy,
            lanes: plan.lane_count,
            alt_m: plan.alt_m as f64,
            speed_m_s: plan.speed_m_s as f64,
            spacing_m: plan.effective_spacing_m,
            plan_amps: plan.battery.avg_current_a,
            capacity_mah: plan.battery.capacity_mah,
            reserve_pct: plan.battery.reserve_pct,
            s: 0.0,
            started: false,
            auto_ticks: 0,
            amps_ema: None,
            strikes: 0,
            rtl_commanded: false,
            noted_start: false,
            noted_done: false,
            last_need: None,
            last_usable: None,
            last_report: None,
        }
    }

    fn total(&self) -> f64 {
        self.path.total()
    }

    /// How far off the route a fix may be and still count as being on it. Under half the lane
    /// spacing, so a noisy fix cannot be mistaken for the neighbouring lane.
    fn max_off_m(&self) -> f64 {
        (self.spacing_m * 0.45).clamp(3.0, 25.0)
    }

    /// Progress only starts once the aircraft actually arrives at the first lane, so the transit
    /// from home cannot eat lanes it merely flies over.
    fn start_radius_m(&self) -> f64 {
        (self.max_off_m() * 2.0).max(8.0)
    }

    /// Update progress from a position fix while the vehicle is flying the mission.
    fn advance(&mut self, pos: LatLon) {
        let p = self.frame.to_xy(pos);
        if !self.started {
            let start = self.path.point_at(0.0);
            if (p.0 - start.0).hypot(p.1 - start.1) > self.start_radius_m() {
                return;
            }
            self.started = true;
        }
        let lookahead = self.spacing_m * 2.0 + 60.0;
        if let Some((s, _)) = self
            .path
            .project(p, self.s, self.s + lookahead, self.max_off_m())
            && s > self.s
        {
            self.s = s;
        }
    }

    fn lanes_done(&self) -> usize {
        (0..self.lanes)
            .filter(|lane| self.path.cum_at(lane * 2 + 1) <= self.s + 0.01)
            .count()
    }

    fn phase(&self, mode: &str) -> Phase {
        let heading_home = matches!(mode, "RTL" | "SMART_RTL" | "AUTO_RTL" | "LAND");
        if self.s >= self.total() - DONE_M {
            Phase::Done
        } else if heading_home && (self.s > 0.0 || self.rtl_commanded) {
            Phase::Rtl
        } else if self.s > 0.0 {
            Phase::Sweeping
        } else {
            Phase::Transit
        }
    }

    fn remaining(&self, pos: LatLon) -> Remaining {
        let p = self.frame.to_xy(pos);
        let on_route = self.path.point_at(self.s);
        let to_route = (p.0 - on_route.0).hypot(p.1 - on_route.1);
        let end = self.path.point_at(self.total());
        Remaining {
            path_m: (self.total() - self.s) + to_route,
            turns: self
                .lanes
                .saturating_sub(self.lanes_done())
                .saturating_sub(1),
            back_m: (end.0 - self.home_xy.0).hypot(end.1 - self.home_xy.1),
            alt_m: self.alt_m,
            speed_m_s: self.speed_m_s,
        }
    }

    /// Ask "can I still finish and get home?" using the larger of the planned current and the
    /// current actually being drawn.
    fn check_battery(&mut self, pos: LatLon, battery_pct: f64) -> BatteryCheck {
        let amps = self
            .amps_ema
            .map_or(self.plan_amps, |live| live.max(self.plan_amps));
        let need_mah = remaining_need_mah(&self.remaining(pos), amps);
        let usable_mah = usable_now_mah(self.capacity_mah, battery_pct, self.reserve_pct);
        if must_return(need_mah, usable_mah) {
            self.strikes = self.strikes.saturating_add(1);
        } else {
            self.strikes = 0;
        }
        self.last_need = Some(need_mah);
        self.last_usable = Some(usable_mah);
        BatteryCheck {
            need_mah,
            usable_mah,
            trigger: self.strikes >= STRIKES_NEEDED,
        }
    }

    fn progress(&self, mode: &str) -> SweepProgress {
        SweepProgress {
            plan_id: self.plan_id,
            phase: self.phase(mode),
            s_m: self.s,
            total_m: self.total(),
            lane: self.lanes_done(),
            lanes: self.lanes,
            need_mah: self.last_need,
            usable_mah: self.last_usable,
            rtl_commanded: self.rtl_commanded,
        }
    }
}

pub async fn run(state: AppState) {
    let mut ticker = interval(TICK);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut tracker: Option<Tracker> = None;
    loop {
        ticker.tick().await;
        step(&state, &mut tracker).await;
    }
}

fn note(state: &AppState, level: &str, message: String) {
    let _ = state.tx.send(ServerMessage::SweepNote {
        level: level.to_string(),
        message,
    });
}

async fn step(state: &AppState, tracker: &mut Option<Tracker>) {
    let Some(plan) = state.sweep.read().await.clone() else {
        *tracker = None;
        return;
    };
    if tracker.as_ref().is_none_or(|t| t.plan_id != plan.id) {
        *tracker = Some(Tracker::new(&plan));
    }
    let Some(t) = tracker.as_mut() else { return };
    let Some(tel) = state.latest.read().await.clone() else {
        return;
    };
    if now_ms().saturating_sub(tel.timestamp_ms) > STALE_MS {
        return;
    }
    let has_fix = tel.lat.is_finite() && tel.lon.is_finite() && !(tel.lat == 0.0 && tel.lon == 0.0);
    let pos = LatLon {
        lat: tel.lat,
        lon: tel.lon,
    };

    let mut check = None;
    if tel.armed && tel.flight_mode == "AUTO" && has_fix {
        t.auto_ticks = t.auto_ticks.saturating_add(1);
        if t.auto_ticks > AMPS_EMA_AFTER_TICKS
            && let Some(amps) = tel.current_a
            && amps > 0.5
        {
            let amps = amps as f64;
            t.amps_ema = Some(
                t.amps_ema
                    .map_or(amps, |old| old * (1.0 - AMPS_EMA_ALPHA) + amps * AMPS_EMA_ALPHA),
            );
        }
        t.advance(pos);
        if plan.auto_rtl
            && !t.rtl_commanded
            && t.auto_ticks > SETTLE_TICKS
            && t.s < t.total() - DONE_M
            && let Some(pct) = tel.battery_pct
        {
            check = Some(t.check_battery(pos, pct as f64));
        }
    }

    if let Some(check) = check
        && check.trigger
    {
        t.rtl_commanded = true;
        let message = format!(
            "battery can't finish this sweep ◈ {:.0} mAh needed (x1.2) vs {:.0} usable ◈ turning home after {}/{} lanes",
            check.need_mah,
            check.usable_mah.max(0.0),
            t.lanes_done(),
            t.lanes
        );
        warn!(plan = t.plan_id, "{message}");
        note(state, "warning", message);
        let state = state.clone();
        tokio::spawn(async move {
            match vehicle::command_rtl(&state).await {
                Ok(()) => note(&state, "nominal", "RTL confirmed ◈ neco is coming home".into()),
                Err(error) => note(
                    &state,
                    "danger",
                    format!("RTL NOT confirmed ({error}) ◈ take over manually now"),
                ),
            }
        });
    }

    let progress = t.progress(&tel.flight_mode);
    if progress.phase == Phase::Sweeping && !t.noted_start {
        t.noted_start = true;
        info!(plan = t.plan_id, "sweep underway");
        note(state, "nominal", "neco found the first lane ◈ nom nom nom".into());
    }
    if progress.phase == Phase::Done && !t.noted_done {
        t.noted_done = true;
        info!(plan = t.plan_id, "sweep complete");
        note(
            state,
            "nominal",
            format!(
                "sweep complete ◈ neco is full ◈ {}/{} lanes eaten",
                progress.lane, progress.lanes
            ),
        );
    }

    let due = match &t.last_report {
        None => true,
        Some((at, prev)) => {
            prev.phase != progress.phase
                || prev.lane != progress.lane
                || prev.rtl_commanded != progress.rtl_commanded
                || (progress.s_m - prev.s_m).abs() >= REPORT_STEP_M
                || (at.elapsed() >= REPORT_EVERY && *prev != progress)
        }
    };
    if due {
        *state.sweep_progress.write().await = Some(progress.clone());
        let _ = state.tx.send(ServerMessage::SweepProgress {
            progress: progress.clone(),
        });
        t.last_report = Some((Instant::now(), progress));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        mission::HomePosition,
        sweep::{
            estimate::{BatteryInput, Route, estimate},
            plan::plan_lanes,
        },
    };

    /// 100 m x 60 m area, east-west lanes 20 m apart: three 100 m lanes joined by two 20 m hops.
    /// Route length 340 m; lane ends fall at 100, 220 and 340 m.
    fn test_plan() -> SweepPlan {
        let frame = Frame::new(LatLon { lat: 26.4, lon: 50.1 });
        let polygon: Vec<LatLon> = [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)]
            .iter()
            .map(|&p| frame.to_latlon(p))
            .collect();
        let lanes = plan_lanes(&polygon, 20.0, Some(90.0)).unwrap();
        assert_eq!(lanes.lane_count, 3);
        let home = frame.to_latlon((0.0, -30.0));
        let route = Route {
            sweep_m: lanes.path_m,
            turns: lanes.turns,
            out_m: 80.0,
            back_m: 100.0,
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
            id: 7,
            path: lanes.path,
            lane_count: lanes.lane_count,
            angle_deg: lanes.angle_deg,
            spacing_m: 20.0,
            effective_spacing_m: lanes.effective_spacing_m,
            alt_m: 30.0,
            speed_m_s: 5.0,
            home: HomePosition {
                lat: home.lat,
                lon: home.lon,
                alt_m: 10.0,
            },
            path_m: lanes.path_m,
            out_m: 80.0,
            back_m: 100.0,
            battery,
            auto_rtl: true,
            estimate: estimate(&route, &battery),
        }
    }

    fn at(t: &Tracker, s: f64) -> LatLon {
        t.frame.to_latlon(t.path.point_at(s))
    }

    #[test]
    fn progress_follows_the_lanes_and_never_goes_backwards() {
        let mut t = Tracker::new(&test_plan());
        assert!((t.total() - 340.0).abs() < 0.01);
        let mut last = 0.0;
        let mut s = 0.0;
        while s <= 340.0 {
            let pos = at(&t, s);
            t.advance(pos);
            assert!(t.s >= last, "progress went backwards at {s}");
            assert!((t.s - s).abs() < 1.0, "expected about {s}, got {}", t.s);
            last = t.s;
            s += 5.0;
        }
        assert_eq!(t.lanes_done(), 3);
        assert_eq!(t.phase("AUTO"), Phase::Done);
        // Going back to the start later does not un-eat anything.
        let start = at(&t, 0.0);
        t.advance(start);
        assert!(t.s > 330.0);
    }

    #[test]
    fn the_transit_from_home_does_not_eat_lanes() {
        let mut t = Tracker::new(&test_plan());
        // Home is 30 m south of the area; the first lane starts at the north-west corner region.
        let home = t.frame.to_latlon(t.home_xy);
        t.advance(home);
        assert_eq!(t.s, 0.0);
        assert_eq!(t.phase("AUTO"), Phase::Transit);
        // A fly-over of the middle of lane 3 on the way in must not count either.
        let over_lane_three = at(&t, 290.0);
        t.advance(over_lane_three);
        assert_eq!(t.s, 0.0);
        // Arriving at the first lane starts it.
        let start = at(&t, 3.0);
        t.advance(start);
        assert!(t.s > 0.0);
        assert_eq!(t.phase("AUTO"), Phase::Sweeping);
    }

    #[test]
    fn a_noisy_fix_cannot_hop_to_the_next_lane() {
        let mut t = Tracker::new(&test_plan());
        for s in [0.0, 10.0, 20.0, 30.0] {
            let pos = at(&t, s);
            t.advance(pos);
        }
        assert!((t.s - 30.0).abs() < 1.0);
        // 12 m sideways towards lane 2 (lane spacing is 20 m): not on lane 1, and lane 2 is
        // further along the route than we are allowed to jump.
        let (x, y) = t.path.point_at(30.0);
        let off_route = t.frame.to_latlon((x, y - 12.0));
        t.advance(off_route);
        assert!((t.s - 30.0).abs() < 1.0);
        // A fix a few metres off the lane still counts.
        let near = t.frame.to_latlon((x + 2.0, y - 4.0));
        t.advance(near);
        assert!((t.s - 32.0).abs() < 1.0);
    }

    #[test]
    fn lanes_done_counts_finished_lanes() {
        let mut t = Tracker::new(&test_plan());
        for s in (0..=130).step_by(5) {
            let pos = at(&t, s as f64);
            t.advance(pos);
        }
        assert_eq!(t.lanes_done(), 1);
        assert_eq!(t.phase("AUTO"), Phase::Sweeping);
        assert_eq!(t.phase("RTL"), Phase::Rtl);
        assert_eq!(t.phase("LAND"), Phase::Rtl);
    }

    #[test]
    fn the_battery_check_needs_a_sustained_shortfall() {
        let mut t = Tracker::new(&test_plan());
        let start = at(&t, 0.0);
        t.advance(start);
        // Full pack: plenty of margin, never triggers.
        for _ in 0..10 {
            assert!(!t.check_battery(start, 100.0).trigger);
        }
        // Barely above the reserve: the shortfall has to persist for several ticks.
        for _ in 0..(STRIKES_NEEDED - 1) {
            assert!(!t.check_battery(start, 33.0).trigger);
        }
        assert!(t.check_battery(start, 33.0).trigger);
        // One good reading resets the count.
        let mut t = Tracker::new(&test_plan());
        for _ in 0..(STRIKES_NEEDED - 1) {
            t.check_battery(start, 33.0);
        }
        assert!(!t.check_battery(start, 100.0).trigger);
        assert!(!t.check_battery(start, 33.0).trigger);
    }

    #[test]
    fn live_current_above_the_plan_makes_the_check_stricter() {
        let mut t = Tracker::new(&test_plan());
        let start = at(&t, 0.0);
        let planned = t.check_battery(start, 100.0).need_mah;
        t.amps_ema = Some(40.0);
        let live = t.check_battery(start, 100.0).need_mah;
        assert!((live / planned - 2.0).abs() < 1e-9);
        // A live figure below the plan never makes it more relaxed than the plan.
        t.amps_ema = Some(5.0);
        assert!((t.check_battery(start, 100.0).need_mah - planned).abs() < 1e-9);
    }

    #[test]
    fn remaining_work_shrinks_as_lanes_are_eaten() {
        let mut t = Tracker::new(&test_plan());
        let start = at(&t, 0.0);
        t.advance(start);
        let before = t.remaining(start);
        assert!((before.path_m - 340.0).abs() < 0.5 && before.turns == 2);
        for s in (0..=230).step_by(5) {
            let pos = at(&t, s as f64);
            t.advance(pos);
        }
        let now = at(&t, 230.0);
        let after = t.remaining(now);
        assert!(after.path_m < 120.0 && after.turns == 0);
    }

    #[test]
    fn progress_reports_the_numbers_the_map_needs() {
        let mut t = Tracker::new(&test_plan());
        let start = at(&t, 0.0);
        t.advance(start);
        t.check_battery(start, 100.0);
        let p = t.progress("AUTO");
        assert_eq!(p.plan_id, 7);
        assert_eq!((p.lane, p.lanes), (0, 3));
        assert!(p.need_mah.is_some() && p.usable_mah == Some(3500.0));
        assert!(!p.rtl_commanded);
    }
}
