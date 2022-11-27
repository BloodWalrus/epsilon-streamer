use ecore::EpsilonResult;
use glam::Vec3A;
use serde::Deserialize;

use std::{error::Error, fmt::Display, net::SocketAddr, time::Duration};

use crate::SENSOR_COUNT;

#[derive(Debug)]
pub enum ConfigError {
    NoValidSocketAddrs,
    TooManyDevices,
    NotEnoughDevices,
    FrequencyIsZero,
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("config invalid: ")?;
        f.write_str(match self {
            ConfigError::NoValidSocketAddrs => {
                "config contains no combination of port and ip addr that the streamer can bind to"
            }
            ConfigError::TooManyDevices => "config contains too many devices",
            ConfigError::NotEnoughDevices => "config does not contain enough devices",
            ConfigError::FrequencyIsZero => "frequency cannot be zero",
        })?;
        Ok(())
    }
}

impl Error for ConfigError {}

pub fn validate_config(config: &Config) -> EpsilonResult<()> {
    if config.devices.len() > SENSOR_COUNT {
        return Err(Box::new(ConfigError::TooManyDevices));
    } else if config.devices.len() < SENSOR_COUNT {
        return Err(Box::new(ConfigError::NotEnoughDevices));
    }

    if config.sockets.len() == 0 {
        return Err(Box::new(ConfigError::NoValidSocketAddrs));
    }

    if config.frequency == 0.0 {
        return Err(Box::new(ConfigError::FrequencyIsZero));
    }

    Ok(())
}

#[derive(Deserialize)]
pub struct Config {
    pub frequency: f64, // in secs
    pub sockets: Vec<SocketAddr>,
    pub devices: Vec<Device>,
}

#[derive(Deserialize)]
pub struct Device {
    pub bus: u8,
    pub addr: u8,
}
