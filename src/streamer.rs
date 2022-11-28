// all this code needs cleaning up

use crate::{
    config::{validate_config, Config, ConfigError},
    sensor::{FromDevice, SensorArray},
    Gyro, QUAT_ARRAY_SIZE,
};
use ecore::{connection::Streamer, constants::*, EpsilonResult};
use glam::Quat;
use std::{
    error::Error,
    fmt::Display,
    thread,
    time::{Duration, Instant},
};

#[derive(Debug)]
pub enum StreamerError {
    ConfigInvalid(ConfigError),
    DevicesInvalid,
}

impl Display for StreamerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let tmp;
        f.write_str(match self {
            StreamerError::ConfigInvalid(config_error) => {
                tmp = format!("config error: {}", config_error);
                &tmp
            }
            StreamerError::DevicesInvalid => "not all devices in config are valid",
        })
    }
}

impl Error for StreamerError {}

pub struct EpsilonStreamer {
    sensor_array: SensorArray<SENSOR_COUNT>,
    streamer: Streamer<[Quat; SENSOR_COUNT], QUAT_ARRAY_SIZE>,
    frequency: Duration, //measured in time per frame
}

impl EpsilonStreamer {
    pub fn new() -> EpsilonResult<Self> {
        let config_data = std::fs::read(CONFIG_PATH)?;
        let config: Config = toml::from_slice(&config_data)?;
        // validate config should crash the program if no valid config is provided. this is system software not application software.
        // if what you give it isn't right it will not give you a real time prompt to correct it. !!! make sure the config is right !!!
        validate_config(&config)?;
        let streamer = Streamer::listen(&config.sockets[..])?;
        let mut devices_iter = config
            .devices
            .into_iter()
            .map(Gyro::from_device)
            .filter_map(|device| match device {
                Ok(device) => Some(device),
                Err(_) => None,
            });

        let mut devices: [Gyro; SENSOR_COUNT];
        unsafe {
            devices = std::mem::zeroed(); // livin on the edge
            for i in 0..SENSOR_COUNT {
                let tmp = if let Some(device) = devices_iter.next() {
                    device
                } else {
                    return Err(Box::new(StreamerError::DevicesInvalid));
                };

                std::ptr::write((&mut devices[i]) as *mut Gyro, tmp);
            }
        }

        let sensor_array = SensorArray::new(devices);

        Ok(Self {
            sensor_array,
            streamer,
            frequency: Duration::from_secs_f64(config.frequency),
        })
    }

    pub fn run(mut self) -> EpsilonResult<()> {
        let mut delta = Duration::ZERO;
        let mut tmp = [Quat::IDENTITY; SENSOR_COUNT];

        'client: loop {
            if let Err(err) = self.streamer.next_client() {
                eprintln!("{}", err);
                continue 'client;
            }

            'stream: loop {
                let timer = Instant::now();
                self.sensor_array.read(&mut tmp);

                if let Err(err) = self.streamer.send(tmp.clone()) {
                    eprintln!("{}", err);
                    continue 'client;
                }

                delta = timer.elapsed();

                let tmp = self.frequency.checked_sub(delta).unwrap_or(Duration::ZERO);
                if tmp > Duration::ZERO {
                    thread::sleep(tmp);
                }
            }
        }
    }
}
