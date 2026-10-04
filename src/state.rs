use crate::{
    mission::{HomePosition, MissionEvent, MissionPlan},
    telemetry::{DataSource, ServerMessage, Telemetry},
};
use mavlink::{AsyncMavConnection, dialects::ardupilotmega::MavMessage};
use std::sync::Arc;
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
        }
    }
}
