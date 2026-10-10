//! Battery and time budget for a sweep. Pure maths, no I/O, so it is easy to test.
//!
//! The model is deliberately simple and honest: one average current figure for the whole flight,
//! plus fixed allowances for turns, climb and landing. Use a measured current for the aircraft you
//! are actually flying; a guess will be wrong.

use serde::{Deserialize, Serialize};

/// Slow-down and re-acceleration allowance at each lane turn.
pub const TURN_PENALTY_S: f64 = 2.0;
pub const CLIMB_M_S: f64 = 2.5;
pub const DESCENT_M_S: f64 = 2.0;
/// The last few metres of an RTL/land are slow.
pub const FINAL_LANDING_S: f64 = 20.0;
/// Using more than this share of the usable battery is flagged as "tight".
pub const TIGHT_FRACTION: f64 = 0.85;
pub const DEFAULT_RESERVE_PCT: f64 = 30.0;
/// In flight, require this much headroom over the estimate before carrying on.
pub const MONITOR_MARGIN: f64 = 1.2;

fn default_reserve() -> f64 {
    DEFAULT_RESERVE_PCT
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct BatteryInput {
    pub capacity_mah: f64,
    pub remaining_pct: f64,
    pub avg_current_a: f64,
    #[serde(default = "default_reserve")]
    pub reserve_pct: f64,
}

impl BatteryInput {
    pub fn validate(&self) -> Result<(), String> {
        if !self.capacity_mah.is_finite() || !(100.0..=100_000.0).contains(&self.capacity_mah) {
            return Err("battery capacity must be 100–100000 mAh".into());
        }
        if !self.remaining_pct.is_finite() || !(0.0..=100.0).contains(&self.remaining_pct) {
            return Err("battery remaining must be 0–100 %".into());
        }
        if !self.avg_current_a.is_finite() || !(0.5..=500.0).contains(&self.avg_current_a) {
            return Err("average current must be 0.5–500 A".into());
        }
        if !self.reserve_pct.is_finite() || !(0.0..=90.0).contains(&self.reserve_pct) {
            return Err("reserve must be 0–90 %".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    Ok,
    Tight,
    TooLong,
}

/// The distances that make up one sortie.
#[derive(Debug, Clone, Copy)]
pub struct Route {
    /// Metres along the lanes, including the hops between them.
    pub sweep_m: f64,
    pub turns: usize,
    /// Home to the first lane.
    pub out_m: f64,
    /// Last lane back to home.
    pub back_m: f64,
    pub alt_m: f64,
    pub speed_m_s: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct Estimate {
    pub sweep_m: f64,
    pub transit_m: f64,
    pub total_m: f64,
    pub flight_s: f64,
    pub need_mah: f64,
    pub available_mah: f64,
    pub reserve_mah: f64,
    /// Available minus reserve. Negative means the pack is already below the reserve.
    pub usable_mah: f64,
    pub margin_mah: f64,
    pub used_pct_of_usable: Option<f64>,
    /// How much of this sweep one battery covers (100 = all of it).
    pub coverage_pct: f64,
    /// Batteries needed to cover the whole area, or `None` if the pack is already under reserve.
    pub sorties: Option<u32>,
    pub verdict: Verdict,
}

impl Estimate {
    /// One-line explanation, used for upload refusals and logs.
    pub fn describe(&self) -> String {
        format!(
            "needs {:.0} mAh (~{:.1} min) of {:.0} mAh usable",
            self.need_mah,
            self.flight_s / 60.0,
            self.usable_mah.max(0.0)
        )
    }
}

pub fn flight_time_s(route: &Route) -> f64 {
    (route.sweep_m + route.out_m + route.back_m) / route.speed_m_s
        + route.turns as f64 * TURN_PENALTY_S
        + route.alt_m / CLIMB_M_S
        + landing_s(route.alt_m)
}

fn landing_s(alt_m: f64) -> f64 {
    alt_m / DESCENT_M_S + FINAL_LANDING_S
}

pub fn mah_for(seconds: f64, avg_current_a: f64) -> f64 {
    avg_current_a * 1000.0 * seconds / 3600.0
}

pub fn estimate(route: &Route, battery: &BatteryInput) -> Estimate {
    let flight_s = flight_time_s(route);
    let need_mah = mah_for(flight_s, battery.avg_current_a);
    let available_mah = battery.capacity_mah * battery.remaining_pct / 100.0;
    let reserve_mah = battery.capacity_mah * battery.reserve_pct / 100.0;
    let usable_mah = available_mah - reserve_mah;
    let verdict = if usable_mah <= 0.0 || need_mah > usable_mah {
        Verdict::TooLong
    } else if need_mah > usable_mah * TIGHT_FRACTION {
        Verdict::Tight
    } else {
        Verdict::Ok
    };
    let (used_pct_of_usable, coverage_pct, sorties) = if usable_mah > 0.0 {
        (
            Some(need_mah / usable_mah * 100.0),
            (usable_mah / need_mah * 100.0).min(100.0),
            Some((need_mah / usable_mah).ceil().max(1.0) as u32),
        )
    } else {
        (None, 0.0, None)
    };
    Estimate {
        sweep_m: route.sweep_m,
        transit_m: route.out_m + route.back_m,
        total_m: route.sweep_m + route.out_m + route.back_m,
        flight_s,
        need_mah,
        available_mah,
        reserve_mah,
        usable_mah,
        margin_mah: usable_mah - need_mah,
        used_pct_of_usable,
        coverage_pct,
        sorties,
        verdict,
    }
}

/// What is left of a flight in progress.
#[derive(Debug, Clone, Copy)]
pub struct Remaining {
    /// From where the aircraft is now, along the rest of the lanes.
    pub path_m: f64,
    pub turns: usize,
    /// From the end of the lanes back to home.
    pub back_m: f64,
    pub alt_m: f64,
    pub speed_m_s: f64,
}

/// Energy to finish the lanes, fly home and land. No climb: the aircraft is already up.
pub fn remaining_need_mah(rem: &Remaining, avg_current_a: f64) -> f64 {
    let seconds = (rem.path_m + rem.back_m) / rem.speed_m_s
        + rem.turns as f64 * TURN_PENALTY_S
        + landing_s(rem.alt_m);
    mah_for(seconds, avg_current_a)
}

/// Charge left above the reserve right now. Negative means already inside the reserve.
pub fn usable_now_mah(capacity_mah: f64, remaining_pct: f64, reserve_pct: f64) -> f64 {
    capacity_mah * (remaining_pct - reserve_pct) / 100.0
}

/// True when carrying on would eat into the reserve, with [`MONITOR_MARGIN`] headroom.
pub fn must_return(need_mah: f64, usable_mah: f64) -> bool {
    need_mah * MONITOR_MARGIN > usable_mah
}

#[cfg(test)]
mod tests {
    use super::*;

    fn route() -> Route {
        Route {
            sweep_m: 1000.0,
            turns: 9,
            out_m: 100.0,
            back_m: 100.0,
            alt_m: 30.0,
            speed_m_s: 5.0,
        }
    }

    fn pack(amps: f64) -> BatteryInput {
        BatteryInput {
            capacity_mah: 5000.0,
            remaining_pct: 100.0,
            avg_current_a: amps,
            reserve_pct: 30.0,
        }
    }

    #[test]
    fn flight_time_adds_up() {
        // 1200 m at 5 m/s + 9 turns x 2 s + 30 m climb at 2.5 + 30 m descent at 2 + 20 s landing
        assert!((flight_time_s(&route()) - (240.0 + 18.0 + 12.0 + 15.0 + 20.0)).abs() < 1e-9);
    }

    #[test]
    fn comfortable_flight_is_ok() {
        let e = estimate(&route(), &pack(20.0));
        assert_eq!(e.verdict, Verdict::Ok);
        assert!((e.need_mah - 20.0 * 1000.0 * 305.0 / 3600.0).abs() < 1e-9);
        assert_eq!(e.usable_mah, 3500.0);
        assert_eq!(e.sorties, Some(1));
        assert_eq!(e.coverage_pct, 100.0);
        assert!(e.margin_mah > 0.0);
        assert!((e.total_m - 1200.0).abs() < 1e-9 && (e.transit_m - 200.0).abs() < 1e-9);
    }

    #[test]
    fn nearly_full_battery_is_tight() {
        let e = estimate(&route(), &pack(40.0));
        assert_eq!(e.verdict, Verdict::Tight);
        assert!(e.margin_mah > 0.0);
        assert!(e.used_pct_of_usable.unwrap() > 85.0);
    }

    #[test]
    fn too_long_reports_coverage_and_sorties() {
        let e = estimate(&route(), &pack(60.0));
        assert_eq!(e.verdict, Verdict::TooLong);
        assert!(e.margin_mah < 0.0);
        assert_eq!(e.sorties, Some(2));
        assert!((e.coverage_pct - 3500.0 / e.need_mah * 100.0).abs() < 1e-9);
        assert!(e.describe().contains("usable"));
    }

    #[test]
    fn a_pack_already_under_reserve_cannot_fly() {
        let mut battery = pack(10.0);
        battery.remaining_pct = 25.0;
        let e = estimate(&route(), &battery);
        assert_eq!(e.verdict, Verdict::TooLong);
        assert_eq!(e.sorties, None);
        assert_eq!(e.coverage_pct, 0.0);
        assert!(e.usable_mah < 0.0);
    }

    #[test]
    fn battery_inputs_are_validated() {
        assert!(pack(20.0).validate().is_ok());
        let mut b = pack(20.0);
        b.capacity_mah = 0.0;
        assert!(b.validate().is_err());
        let mut b = pack(20.0);
        b.remaining_pct = 101.0;
        assert!(b.validate().is_err());
        assert!(pack(0.1).validate().is_err());
        assert!(pack(f64::NAN).validate().is_err());
        let mut b = pack(20.0);
        b.reserve_pct = 95.0;
        assert!(b.validate().is_err());
    }

    #[test]
    fn reserve_defaults_when_omitted() {
        let b: BatteryInput =
            serde_json::from_str(r#"{"capacity_mah":4000,"remaining_pct":90,"avg_current_a":15}"#)
                .unwrap();
        assert_eq!(b.reserve_pct, DEFAULT_RESERVE_PCT);
    }

    #[test]
    fn in_flight_decision_leaves_a_margin() {
        let rem = Remaining {
            path_m: 500.0,
            turns: 4,
            back_m: 150.0,
            alt_m: 30.0,
            speed_m_s: 5.0,
        };
        // 650/5 = 130 s + 8 s turns + (15 + 20) s landing = 173 s at 30 A.
        let need = remaining_need_mah(&rem, 30.0);
        assert!((need - 30.0 * 1000.0 * 173.0 / 3600.0).abs() < 1e-9);
        let usable = usable_now_mah(5000.0, 60.0, 30.0);
        assert_eq!(usable, 1500.0);
        // 1441.7 mAh needed; with the 1.2x margin that is 1730 > 1500, so turn back.
        assert!(must_return(need, usable));
        // A fuller pack can carry on.
        assert!(!must_return(need, usable_now_mah(5000.0, 80.0, 30.0)));
        // Already inside the reserve.
        assert!(must_return(1.0, usable_now_mah(5000.0, 20.0, 30.0)));
    }
}
