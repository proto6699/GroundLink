//! Lane ("lawnmower") generation for a polygon survey area.
//!
//! Lanes run along a chosen compass bearing and are stepped sideways across the polygon.
//! Direction alternates every lane (boustrophedon) so the aircraft never flies an empty return leg.

use super::geo::{Frame, LatLon, Path};
use serde::Serialize;

pub const MAX_VERTICES: usize = 64;
/// Lane endpoints that fit in one uploaded mission (two per lane).
pub const MAX_PATH_POINTS: usize = 150;
pub const MIN_SPACING_M: f64 = 2.0;
pub const MAX_SPACING_M: f64 = 200.0;
const MIN_AREA_M2: f64 = 20.0;
const MIN_WIDTH_M: f64 = 1.0;
const MAX_EXTENT_M: f64 = 10_000.0;
const MIN_LANE_M: f64 = 1.0;

#[derive(Debug, Clone, Serialize)]
pub struct LanePlan {
    /// Lane endpoints in flight order: lane `i` runs `path[2i] -> path[2i + 1]`.
    pub path: Vec<LatLon>,
    pub lane_count: usize,
    pub angle_deg: f64,
    pub auto_angle: bool,
    pub spacing_m: f64,
    /// The spacing actually used: the area width divided evenly, never wider than requested.
    pub effective_spacing_m: f64,
    /// Metres flown along lanes.
    pub lane_m: f64,
    /// Metres along the whole route, including the short hops between lanes.
    pub path_m: f64,
    pub turns: usize,
    pub area_m2: f64,
    pub concave: bool,
    pub warnings: Vec<String>,
}

pub fn plan_lanes(
    polygon: &[LatLon],
    spacing_m: f64,
    angle_deg: Option<f64>,
) -> Result<LanePlan, String> {
    if !spacing_m.is_finite() || !(MIN_SPACING_M..=MAX_SPACING_M).contains(&spacing_m) {
        return Err(format!(
            "lane spacing must be {MIN_SPACING_M}–{MAX_SPACING_M} m"
        ));
    }
    if let Some(angle) = angle_deg
        && !angle.is_finite()
    {
        return Err("lane angle must be a number".into());
    }
    if polygon.len() < 3 {
        return Err("draw at least 3 corners".into());
    }
    if polygon.len() > MAX_VERTICES {
        return Err(format!("area is capped at {MAX_VERTICES} corners"));
    }
    if let Some(i) = polygon.iter().position(|p| !p.is_valid()) {
        return Err(format!("corner {} has an invalid coordinate", i + 1));
    }

    let n = polygon.len() as f64;
    let origin = LatLon {
        lat: polygon.iter().map(|p| p.lat).sum::<f64>() / n,
        lon: polygon.iter().map(|p| p.lon).sum::<f64>() / n,
    };
    let frame = Frame::new(origin);
    let xy: Vec<(f64, f64)> = polygon.iter().map(|&p| frame.to_xy(p)).collect();
    if xy.iter().any(|p| p.0.abs() > MAX_EXTENT_M || p.1.abs() > MAX_EXTENT_M) {
        return Err("area is too large (keep it within roughly 10 km)".into());
    }
    if self_intersects(&xy) {
        return Err("area boundary crosses itself".into());
    }
    let area_m2 = shoelace(&xy).abs();
    if area_m2 < MIN_AREA_M2 {
        return Err("area is too small to sweep".into());
    }
    let concave = !is_convex(&xy);

    let (theta, auto_angle) = match angle_deg {
        Some(angle) => (angle.rem_euclid(360.0), false),
        None => (best_angle(&xy), true),
    };
    let axes = Axes::new(theta);
    let uv: Vec<(f64, f64)> = xy.iter().map(|&p| axes.to_uv(p)).collect();

    let v_min = uv.iter().map(|p| p.1).fold(f64::MAX, f64::min);
    let v_max = uv.iter().map(|p| p.1).fold(f64::MIN, f64::max);
    let width = v_max - v_min;
    if width < MIN_WIDTH_M {
        return Err("area is too thin to sweep".into());
    }
    let lane_target = ((width / spacing_m) - 1e-9).ceil().max(1.0) as usize;
    if lane_target * 2 > MAX_PATH_POINTS {
        return Err(format!(
            "that needs {lane_target} lanes (max {}); widen the lane spacing or shrink the area",
            MAX_PATH_POINTS / 2
        ));
    }
    let effective_spacing_m = width / lane_target as f64;

    // (start_u, end_u, v) in flight order.
    let mut lanes: Vec<(f64, f64, f64)> = Vec::new();
    let mut split_rows = false;
    for k in 0..lane_target {
        let v = v_min + (k as f64 + 0.5) * effective_spacing_m;
        let mut crossings: Vec<f64> = Vec::new();
        for i in 0..uv.len() {
            let (a, b) = (uv[i], uv[(i + 1) % uv.len()]);
            if (a.1 <= v && b.1 > v) || (b.1 <= v && a.1 > v) {
                crossings.push(a.0 + (v - a.1) / (b.1 - a.1) * (b.0 - a.0));
            }
        }
        crossings.sort_by(f64::total_cmp);
        let mut row: Vec<(f64, f64)> = crossings
            .chunks_exact(2)
            .map(|pair| (pair[0], pair[1]))
            .filter(|(a, b)| b - a >= MIN_LANE_M)
            .collect();
        split_rows |= row.len() > 1;
        let forward = k % 2 == 0;
        if !forward {
            row.reverse();
        }
        for (a, b) in row {
            lanes.push(if forward { (a, b, v) } else { (b, a, v) });
        }
    }
    if lanes.is_empty() {
        return Err("no lane fits inside that area".into());
    }
    if lanes.len() * 2 > MAX_PATH_POINTS {
        return Err(format!(
            "that needs {} lanes (max {}); widen the lane spacing or shrink the area",
            lanes.len(),
            MAX_PATH_POINTS / 2
        ));
    }

    let mut path_xy = Vec::with_capacity(lanes.len() * 2);
    let mut lane_m = 0.0;
    for &(a, b, v) in &lanes {
        path_xy.push(axes.to_xy((a, v)));
        path_xy.push(axes.to_xy((b, v)));
        lane_m += (b - a).abs();
    }
    let path_m = Path::new(path_xy.clone()).total();
    let path = path_xy.iter().map(|&p| frame.to_latlon(p)).collect();

    let mut warnings = Vec::new();
    if concave {
        warnings.push(
            "area is concave: the hops between lanes can cut across outside your boundary".into(),
        );
    } else if split_rows {
        warnings.push("some lanes were split by the area shape".into());
    }

    Ok(LanePlan {
        path,
        lane_count: lanes.len(),
        angle_deg: theta,
        auto_angle,
        spacing_m,
        effective_spacing_m,
        lane_m,
        path_m,
        turns: lanes.len() - 1,
        area_m2,
        concave,
        warnings,
    })
}

/// Lane-aligned axes: `u` runs along the lanes (compass bearing `theta`), `v` runs to the right.
struct Axes {
    along: (f64, f64),
    right: (f64, f64),
}

impl Axes {
    fn new(theta_deg: f64) -> Self {
        let (s, c) = theta_deg.to_radians().sin_cos();
        Self {
            along: (s, c),
            right: (c, -s),
        }
    }
    fn to_uv(&self, (x, y): (f64, f64)) -> (f64, f64) {
        (
            x * self.along.0 + y * self.along.1,
            x * self.right.0 + y * self.right.1,
        )
    }
    fn to_xy(&self, (u, v): (f64, f64)) -> (f64, f64) {
        (
            u * self.along.0 + v * self.right.0,
            u * self.along.1 + v * self.right.1,
        )
    }
}

/// The lane direction with the narrowest sideways extent: fewest lanes, so fewest turns.
fn best_angle(xy: &[(f64, f64)]) -> f64 {
    let mut best = (f64::MAX, 0.0);
    for i in 0..xy.len() {
        let (a, b) = (xy[i], xy[(i + 1) % xy.len()]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        if dx.hypot(dy) < 1e-6 {
            continue;
        }
        let theta = dx.atan2(dy).to_degrees().rem_euclid(180.0);
        let axes = Axes::new(theta);
        let (lo, hi) = xy.iter().map(|&p| axes.to_uv(p).1).fold(
            (f64::MAX, f64::MIN),
            |(lo, hi), v| (lo.min(v), hi.max(v)),
        );
        if hi - lo < best.0 - 1e-6 {
            best = (hi - lo, theta);
        }
    }
    best.1
}

fn shoelace(xy: &[(f64, f64)]) -> f64 {
    let mut sum = 0.0;
    for i in 0..xy.len() {
        let (a, b) = (xy[i], xy[(i + 1) % xy.len()]);
        sum += a.0 * b.1 - b.0 * a.1;
    }
    sum / 2.0
}

fn is_convex(xy: &[(f64, f64)]) -> bool {
    let mut sign = 0.0_f64;
    for i in 0..xy.len() {
        let (a, b, c) = (xy[i], xy[(i + 1) % xy.len()], xy[(i + 2) % xy.len()]);
        let cross = (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0);
        if cross.abs() < 1e-9 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    true
}

fn self_intersects(xy: &[(f64, f64)]) -> bool {
    let n = xy.len();
    for i in 0..n {
        for j in (i + 1)..n {
            // Edges that share a corner are allowed to touch.
            if j == i + 1 || (i == 0 && j == n - 1) {
                continue;
            }
            if segments_intersect(xy[i], xy[(i + 1) % n], xy[j], xy[(j + 1) % n]) {
                return true;
            }
        }
    }
    false
}

fn segments_intersect(p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), p4: (f64, f64)) -> bool {
    fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    }
    fn within(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
        c.0 >= a.0.min(b.0) && c.0 <= a.0.max(b.0) && c.1 >= a.1.min(b.1) && c.1 <= a.1.max(b.1)
    }
    let (d1, d2) = (orient(p3, p4, p1), orient(p3, p4, p2));
    let (d3, d4) = (orient(p1, p2, p3), orient(p1, p2, p4));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && within(p3, p4, p1))
        || (d2 == 0.0 && within(p3, p4, p2))
        || (d3 == 0.0 && within(p1, p2, p3))
        || (d4 == 0.0 && within(p1, p2, p4))
}

#[cfg(test)]
mod tests {
    use super::super::geo::distance_m;
    use super::*;

    const ORIGIN: LatLon = LatLon { lat: 26.4, lon: 50.1 };

    /// Build a polygon from metre offsets (east, north) around a fixed origin.
    fn poly(points: &[(f64, f64)]) -> Vec<LatLon> {
        let frame = Frame::new(ORIGIN);
        points.iter().map(|&p| frame.to_latlon(p)).collect()
    }

    fn rect(w: f64, h: f64) -> Vec<LatLon> {
        poly(&[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)])
    }

    fn lane_lengths(plan: &LanePlan) -> Vec<f64> {
        plan.path
            .chunks_exact(2)
            .map(|pair| distance_m(pair[0], pair[1]))
            .collect()
    }

    #[test]
    fn square_gets_evenly_spaced_alternating_lanes() {
        let plan = plan_lanes(&rect(100.0, 100.0), 10.0, Some(0.0)).unwrap();
        assert_eq!(plan.lane_count, 10);
        assert_eq!(plan.path.len(), 20);
        // The test polygon is built in a frame at its corner, the planner uses one at its centroid,
        // so allow a millimetre of difference.
        assert!((plan.effective_spacing_m - 10.0).abs() < 1e-3);
        assert!(lane_lengths(&plan).iter().all(|l| (l - 100.0).abs() < 0.01));
        assert_eq!(plan.turns, 9);
        // Bearing 0 = north: lane 1 flies south->north, lane 2 north->south, 10 m to its right.
        let (a, b) = (plan.path[0], plan.path[1]);
        assert!(b.lat > a.lat);
        let (c, d) = (plan.path[2], plan.path[3]);
        assert!(d.lat < c.lat);
        assert!(c.lon > a.lon);
        // Lane 2 ends level with where lane 1 started, one lane spacing to the right.
        assert!((distance_m(a, d) - 10.0).abs() < 0.05);
        // Lane m = 10 lanes x 100 m.
        assert!((plan.lane_m - 1000.0).abs() < 0.01);
    }

    #[test]
    fn bearing_controls_lane_direction() {
        // Bearing 90 = east: lanes run west->east, first lane on the north edge.
        let plan = plan_lanes(&rect(100.0, 60.0), 20.0, Some(90.0)).unwrap();
        assert_eq!(plan.lane_count, 3);
        let (a, b) = (plan.path[0], plan.path[1]);
        assert!(b.lon > a.lon);
        assert!((b.lat - a.lat).abs() < 1e-9);
        assert!(plan.path[2].lat < a.lat);
    }

    #[test]
    fn spacing_is_never_wider_than_requested() {
        // 95 m wide at 10 m spacing -> 10 lanes at 9.5 m rather than 9 lanes with a gap.
        let plan = plan_lanes(&rect(95.0, 50.0), 10.0, Some(0.0)).unwrap();
        assert_eq!(plan.lane_count, 10);
        assert!((plan.effective_spacing_m - 9.5).abs() < 1e-3);
    }

    #[test]
    fn thin_sliver_still_gets_one_centred_lane() {
        let plan = plan_lanes(&rect(100.0, 3.0), 10.0, None).unwrap();
        assert_eq!(plan.lane_count, 1);
        assert!((lane_lengths(&plan)[0] - 100.0).abs() < 0.01);
    }

    #[test]
    fn auto_angle_flies_along_the_long_side() {
        // 200 m east-west x 50 m north-south: east-west lanes need 5 lanes, north-south needs 20.
        let plan = plan_lanes(&rect(200.0, 50.0), 10.0, None).unwrap();
        assert!(plan.auto_angle);
        assert_eq!(plan.lane_count, 5);
        assert!((plan.angle_deg - 90.0).abs() < 0.5);
        assert!(lane_lengths(&plan).iter().all(|l| (l - 200.0).abs() < 0.05));
    }

    #[test]
    fn concave_l_shape_follows_the_boundary() {
        let l_shape = poly(&[
            (0.0, 0.0),
            (100.0, 0.0),
            (100.0, 40.0),
            (40.0, 40.0),
            (40.0, 100.0),
            (0.0, 100.0),
        ]);
        let plan = plan_lanes(&l_shape, 10.0, Some(0.0)).unwrap();
        assert!(plan.concave);
        assert!(plan.warnings.iter().any(|w| w.contains("concave")));
        assert_eq!(plan.lane_count, 10);
        let lengths = lane_lengths(&plan);
        // x = 5..35 are 100 m tall, x = 45..95 are 40 m tall.
        assert!(lengths[..4].iter().all(|l| (l - 100.0).abs() < 0.05));
        assert!(lengths[4..].iter().all(|l| (l - 40.0).abs() < 0.05));
    }

    #[test]
    fn u_shape_splits_rows_into_separate_lanes() {
        let u_shape = poly(&[
            (0.0, 0.0),
            (100.0, 0.0),
            (100.0, 100.0),
            (70.0, 100.0),
            (70.0, 30.0),
            (30.0, 30.0),
            (30.0, 100.0),
            (0.0, 100.0),
        ]);
        let plan = plan_lanes(&u_shape, 10.0, Some(90.0)).unwrap();
        // 3 full-width rows (100 m) + 7 rows crossing two 30 m arms.
        assert!((plan.lane_m - (3.0 * 100.0 + 7.0 * 60.0)).abs() < 0.1);
        assert_eq!(plan.lane_count, 3 + 7 * 2);
    }

    #[test]
    fn rejects_bad_areas() {
        let bowtie = poly(&[(0.0, 0.0), (100.0, 100.0), (100.0, 0.0), (0.0, 100.0)]);
        assert!(plan_lanes(&bowtie, 10.0, None).unwrap_err().contains("crosses itself"));
        assert!(plan_lanes(&rect(100.0, 100.0)[..2], 10.0, None).is_err());
        assert!(plan_lanes(&rect(100.0, 100.0), 0.5, None).is_err());
        assert!(plan_lanes(&rect(100.0, 100.0), f64::NAN, None).is_err());
        assert!(plan_lanes(&rect(100.0, 100.0), 10.0, Some(f64::NAN)).is_err());
        assert!(plan_lanes(&rect(3.0, 3.0), 10.0, None).unwrap_err().contains("too small"));
        let mut bad = rect(100.0, 100.0);
        bad[1].lat = 91.0;
        assert!(plan_lanes(&bad, 10.0, None).is_err());
        // 1 km square at 2 m spacing would be 500 lanes.
        let err = plan_lanes(&rect(1000.0, 1000.0), 2.0, Some(0.0)).unwrap_err();
        assert!(err.contains("max 75"));
    }

    #[test]
    fn angle_is_normalised() {
        let a = plan_lanes(&rect(100.0, 100.0), 20.0, Some(-90.0)).unwrap();
        assert!((a.angle_deg - 270.0).abs() < 1e-9);
        let b = plan_lanes(&rect(100.0, 100.0), 20.0, Some(450.0)).unwrap();
        assert!((b.angle_deg - 90.0).abs() < 1e-9);
    }
}
