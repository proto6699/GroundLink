//! Small-area geometry for Sweep.
//!
//! Everything works in a local east/north metre frame (equirectangular around an origin).
//! That is plenty accurate for survey areas a few kilometres across.

use serde::{Deserialize, Serialize};

const M_PER_DEG_LAT: f64 = 111_320.0;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub fn is_valid(&self) -> bool {
        self.lat.is_finite()
            && self.lon.is_finite()
            && (-90.0..=90.0).contains(&self.lat)
            && (-180.0..=180.0).contains(&self.lon)
    }
}

/// Local tangent-plane frame: x = metres east of the origin, y = metres north.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    origin: LatLon,
    m_per_deg_lon: f64,
}

impl Frame {
    pub fn new(origin: LatLon) -> Self {
        Self {
            origin,
            m_per_deg_lon: M_PER_DEG_LAT * origin.lat.to_radians().cos(),
        }
    }

    pub fn to_xy(&self, p: LatLon) -> (f64, f64) {
        (
            (p.lon - self.origin.lon) * self.m_per_deg_lon,
            (p.lat - self.origin.lat) * M_PER_DEG_LAT,
        )
    }

    pub fn to_latlon(&self, (x, y): (f64, f64)) -> LatLon {
        LatLon {
            lat: self.origin.lat + y / M_PER_DEG_LAT,
            lon: self.origin.lon + x / self.m_per_deg_lon,
        }
    }
}

/// Straight-line ground distance between two nearby points, in metres.
pub fn distance_m(a: LatLon, b: LatLon) -> f64 {
    let frame = Frame::new(a);
    let (x, y) = frame.to_xy(b);
    x.hypot(y)
}

/// A polyline in the local frame with cumulative distances, so progress can be tracked
/// as "metres travelled along the route".
#[derive(Debug, Clone)]
pub struct Path {
    pts: Vec<(f64, f64)>,
    cum: Vec<f64>,
}

impl Path {
    pub fn new(pts: Vec<(f64, f64)>) -> Self {
        let mut cum = Vec::with_capacity(pts.len());
        let mut total = 0.0;
        for (i, p) in pts.iter().enumerate() {
            if i > 0 {
                let q = pts[i - 1];
                total += (p.0 - q.0).hypot(p.1 - q.1);
            }
            cum.push(total);
        }
        Self { pts, cum }
    }

    pub fn total(&self) -> f64 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    /// Distance along the path at vertex `index`.
    pub fn cum_at(&self, index: usize) -> f64 {
        self.cum[index.min(self.cum.len() - 1)]
    }

    pub fn point_at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.total());
        for i in 1..self.pts.len() {
            if s <= self.cum[i] {
                let len = self.cum[i] - self.cum[i - 1];
                let t = if len > 0.0 {
                    (s - self.cum[i - 1]) / len
                } else {
                    0.0
                };
                let (a, b) = (self.pts[i - 1], self.pts[i]);
                return (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
            }
        }
        self.pts[self.pts.len() - 1]
    }

    /// Project `p` onto the path, looking only at the stretch `[from_s, to_s]`.
    ///
    /// Returns `(s, off_m)` for the closest point, or `None` when nothing in the window is
    /// within `max_off_m`. Restricting the window (and refusing to go backwards) is what stops a
    /// noisy GPS fix from hopping onto a neighbouring lane.
    pub fn project(&self, p: (f64, f64), from_s: f64, to_s: f64, max_off_m: f64) -> Option<(f64, f64)> {
        let mut best: Option<(f64, f64)> = None;
        let hi = to_s.min(self.total()).max(from_s);
        for i in 1..self.pts.len() {
            if self.cum[i] < from_s || self.cum[i - 1] > to_s {
                continue;
            }
            let (a, b) = (self.pts[i - 1], self.pts[i]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            if len2 <= f64::EPSILON {
                continue;
            }
            let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0);
            let s_raw = self.cum[i - 1] + t * len2.sqrt();
            // A closest point beyond the window is a different part of the route, not progress.
            if s_raw > hi {
                continue;
            }
            // Slightly behind where we already are just means "no progress", never going back.
            let s = s_raw.max(from_s);
            let foot = (a.0 + dx * t, a.1 + dy * t);
            let off = (p.0 - foot.0).hypot(p.1 - foot.1);
            if off <= max_off_m && best.is_none_or(|(_, best_off)| off < best_off) {
                best = Some((s, off));
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trips() {
        let frame = Frame::new(LatLon { lat: 26.4, lon: 50.1 });
        let p = LatLon { lat: 26.4012, lon: 50.0987 };
        let back = frame.to_latlon(frame.to_xy(p));
        assert!((back.lat - p.lat).abs() < 1e-12 && (back.lon - p.lon).abs() < 1e-12);
    }

    #[test]
    fn a_degree_of_latitude_is_about_111km() {
        let d = distance_m(LatLon { lat: 24.0, lon: 46.0 }, LatLon { lat: 24.001, lon: 46.0 });
        assert!((d - 111.32).abs() < 0.01);
    }

    #[test]
    fn path_walks_and_projects() {
        let path = Path::new(vec![(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (0.0, 10.0)]);
        assert!((path.total() - 210.0).abs() < 1e-9);
        assert_eq!(path.point_at(50.0), (50.0, 0.0));
        assert_eq!(path.point_at(105.0), (100.0, 5.0));
        let (s, off) = path.project((40.0, 1.0), 0.0, 210.0, 5.0).unwrap();
        assert!((s - 40.0).abs() < 1e-9 && (off - 1.0).abs() < 1e-9);
        // Too far from every segment.
        assert!(path.project((40.0, 50.0), 0.0, 210.0, 5.0).is_none());
    }

    #[test]
    fn projection_never_hops_to_a_neighbouring_lane_or_backwards() {
        // Two lanes 10 m apart, flown in opposite directions.
        let path = Path::new(vec![(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (0.0, 10.0)]);
        // Drone is on lane 1 at x=30. The point is geometrically closer to lane 1, but even a
        // fix that is 4 m off still resolves to the window we allow.
        let (s, _) = path.project((30.0, 4.0), 25.0, 60.0, 5.0).unwrap();
        assert!((s - 30.0).abs() < 1e-9);
        // Window ahead of progress excludes lane 2 entirely.
        assert!(path.project((30.0, 10.0), 25.0, 60.0, 3.0).is_none());
        // Never returns less than `from_s`.
        let (s, _) = path.project((10.0, 0.0), 25.0, 60.0, 30.0).unwrap();
        assert!(s >= 25.0);
        // A point sitting right on lane 2 is not progress when lane 2 lies beyond the window:
        // the segment straddles the window but its closest point (s = 180) is out of reach.
        assert!(path.project((30.0, 10.0), 25.0, 130.0, 3.0).is_none());
        // Once the window reaches it, it counts.
        let (s, _) = path.project((30.0, 10.0), 25.0, 200.0, 3.0).unwrap();
        assert!((s - 180.0).abs() < 1e-9);
    }
}
