use thiserror::Error;

#[derive(Debug, Error)]
pub enum GroundLinkError {
    #[error("MAVLink connection error: {0}")]
    MavlinkConnect(String),
}
