//! Turning a sweep into a MAVLink mission and sending it.

use super::SweepPlan;
use crate::{mission, state::AppState};
use mavlink::dialects::ardupilotmega::{
    MISSION_ITEM_INT_DATA, MavCmd, MavFrame, MavMissionType,
};

/// Mission layout: home placeholder, takeoff, speed, every lane endpoint, then RTL.
///
/// Sequence zero is home because ArduPilot reserves it. The mission ends with an explicit RTL, so
/// finishing the last lane brings the aircraft home instead of hovering at the far corner.
pub fn sweep_items(
    plan: &SweepPlan,
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
            z: plan.alt_m,
            param4: f32::NAN,
            ..base.clone()
        },
        MISSION_ITEM_INT_DATA {
            command: MavCmd::MAV_CMD_DO_CHANGE_SPEED,
            param1: 1.0,
            param2: plan.speed_m_s,
            param3: -1.0,
            ..base.clone()
        },
    ];
    for point in &plan.path {
        items.push(MISSION_ITEM_INT_DATA {
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            param2: 2.0,
            param4: f32::NAN,
            x: (point.lat * 1e7).round() as i32,
            y: (point.lon * 1e7).round() as i32,
            z: plan.alt_m,
            ..base.clone()
        });
    }
    items.push(MISSION_ITEM_INT_DATA {
        command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
        frame: MavFrame::MAV_FRAME_MISSION,
        ..base
    });
    for (seq, item) in items.iter_mut().enumerate() {
        item.seq = seq as u16;
    }
    items
}

pub async fn upload_to_vehicle(state: &AppState, plan: &SweepPlan) -> Result<String, String> {
    if !*state.connected.read().await {
        return Err("vehicle is not connected".into());
    }
    if state.latest.read().await.as_ref().is_some_and(|t| t.armed) {
        return Err("disarm before uploading a sweep".into());
    }
    let message = format!(
        "ArduPilot accepted the sweep: {} lanes at {} m/s, ending in RTL. Arm and select AUTO to fly",
        plan.lane_count, plan.speed_m_s
    );
    mission::transfer_with(state, |system, component| sweep_items(plan, system, component), message)
        .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sweep::sample_plan;

    #[test]
    fn mission_is_home_takeoff_speed_lanes_rtl() {
        let items = sweep_items(&sample_plan(), 1, 1);
        assert_eq!(items.len(), 3 + 4 + 1);
        assert_eq!(items[0].command, MavCmd::MAV_CMD_NAV_WAYPOINT);
        assert_eq!(items[0].frame, MavFrame::MAV_FRAME_GLOBAL);
        assert_eq!(items[0].z, 600.0);
        assert_eq!(items[1].command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert_eq!(items[1].z, 30.0);
        assert_eq!(items[2].command, MavCmd::MAV_CMD_DO_CHANGE_SPEED);
        assert_eq!(items[2].param2, 5.0);
        // Lane endpoints keep their order and the relative-altitude frame.
        assert_eq!(items[3].x, 247000000);
        assert_eq!(items[4].x, 247010000);
        assert_eq!(items[5].y, 466001000);
        assert!(items[3..7].iter().all(|i| i.command == MavCmd::MAV_CMD_NAV_WAYPOINT
            && i.frame == MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
            && i.z == 30.0));
        let last = items.last().unwrap();
        assert_eq!(last.command, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH);
        assert!(items.iter().enumerate().all(|(i, item)| item.seq as usize == i));
        assert!(items.iter().all(|i| i.target_system == 1 && i.target_component == 1));
    }
}
