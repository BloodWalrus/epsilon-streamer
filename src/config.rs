use serde::Deserialize;
use std::{error::Error, fmt::Display, net::IpAddr};

#[derive(Debug)]
pub enum ConfigError {
    NoValidSocketAddrs,
    TooManyDevices,
    NotEnoughDevices,
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ConfigError::NoValidSocketAddrs => {
                "config contains no combination of port and ip addr that the streamer can bind to"
            }
            ConfigError::TooManyDevices => "config contains too many devices",
            ConfigError::NotEnoughDevices => "config does not contain enough devices",
        })
    }
}

impl Error for ConfigError {}

#[derive(Deserialize)]
pub struct Config {
    pub port: u16,
    pub ip_addresses: Vec<IpAddr>,
    pub devices: Vec<Device>,
}

#[derive(Deserialize)]
pub struct Device {
    pub bus: u8,
    pub addr: u8,
}
