use crate::{
    mission::{HomePosition, MissionEvent, MissionPlan},
    sweep::{SweepPlan, SweepProgress},
    telemetry::{DataSource, ServerMessage, Telemetry},
};
use mavlink::{AsyncMavConnection, dialects::ardupilotmega::MavMessage};
use std::sync::{Arc, atomic::AtomicBool};
use tokio::sync::{Mutex, RwLock, broadcast};

pub type SharedMavlinkConnection = Arc<dyn AsyncMavConnection<MavMessage> + Sync + Send>;

#[derive(Clone)]
pub struct AppState {
    pub tx: broadcast::Sender<ServerMessage>,
    pub latest: Arc<RwLock<Option<Telemetry>>>,
    pub connected: Arc<RwLock<bool>>,
    pub source: Arc<RwLock<DataSource>>,
    pub mission: Arc<RwLock<Option<MissionPlan>>>,
    pub home: Arc<RwLock<Option<HomePosition>>>,
    pub mission_upload: Arc<Mutex<()>>,
    pub mission_events: broadcast::Sender<MissionEvent>,
    pub mavlink_connection: Arc<RwLock<Option<SharedMavlinkConnection>>>,
    pub vehicle_target: Arc<RwLock<Option<(u8, u8)>>>,
    /// Sweep (survey-area planner): whether it is switched on, the sweep being watched, and the
    /// last progress report sent to browsers.
    pub sweep_enabled: bool,
    pub sweep: Arc<RwLock<Option<SweepPlan>>>,
    pub sweep_progress: Arc<RwLock<Option<SweepProgress>>>,
    /// Demo mode only: Sweep asked the simulated vehicle to return home.
    pub sim_rtl: Arc<AtomicBool>,
}

impl AppState {
    pub fn new(capacity: usize, source: DataSource) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        let (mission_events, _) = broadcast::channel(64);

        Self {
            tx,
            latest: Arc::new(RwLock::new(None)),
            connected: Arc::new(RwLock::new(false)),
            source: Arc::new(RwLock::new(source)),
            mission: Arc::new(RwLock::new(None)),
            home: Arc::new(RwLock::new(None)),
            mission_upload: Arc::new(Mutex::new(())),
            mission_events,
            mavlink_connection: Arc::new(RwLock::new(None)),
            vehicle_target: Arc::new(RwLock::new(None)),
            sweep_enabled: crate::sweep::enabled_from_env(),
            sweep: Arc::new(RwLock::new(None)),
            sweep_progress: Arc::new(RwLock::new(None)),
            sim_rtl: Arc::new(AtomicBool::new(false)),
        }
    }
}
