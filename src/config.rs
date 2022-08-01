use std::{error::Error, fmt::Display, net::IpAddr};

#[derive(Debug)]
pub enum ConfigError {
    NoValidSocketAddrs,
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ConfigError::NoValidSocketAddrs => {
                "config contains no combination of port and ip addr that the streamer can bind to"
            }
        })
    }
}

impl Error for ConfigError {}

pub struct Config {
    pub port: u16,
    pub ip_addresses: Vec<IpAddr>,
    pub devices: Vec<Device>,
}

pub struct Device {
    pub bus: u8,
    pub addr: u8,
}
