// all this code needs cleaning up
use crate::{
    config::{validate_config, Config, ConfigError},
    sensor::{FromDevice, SensorArray},
    Gyro,
};
use ecore::{
    connection::{CtrlSignal, Stream},
    constants::*,
    EpsilonResult,
};
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
    server: (Stream<[Quat; SENSOR_COUNT]>, Stream<CtrlSignal>),
    frequency: Duration, //measured in time per frame
}

impl EpsilonStreamer {
    pub fn new() -> EpsilonResult<Self> {
        let config_data = std::fs::read(CONFIG_PATH)?;
        let config: Config = toml::from_slice(&config_data)?;
        // validate config should crash the program if no valid config is provided. this is system software not application software.
        // if what you give it isn't right it will not give you a real time prompt to correct it. !!! make sure the config is right !!!
        // don't complain that it crashes, it would be far more irritating if every time you misspelled the config it said "there is an error in the config please retype it to continue:"
        validate_config(&config)?;
        let server = (
            Stream::connect(config.server_data)?,
            Stream::connect(config.server_ctrl)?,
        );

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
            server,
            frequency: Duration::from_secs_f64(config.frequency),
        })
    }

    pub fn run(mut self) -> EpsilonResult<()> {
        let mut delta = Duration::ZERO;
        let mut tmp = [Quat::IDENTITY; SENSOR_COUNT];

        while match self.server.1.recv()? 
        {
            CtrlSignal::Start => false,
            _ => true,
        } {}
        self.sensor_array.start(); 

        loop {
            if let Some(ctrl) = self.server.1.try_recv()? {
                match ctrl {
                    CtrlSignal::Start => {
                        self.sensor_array.start();
                    }
                    // close program returning no error
                    CtrlSignal::Stop => {
                        break Ok(());
                    }
                    CtrlSignal::Reset => {
                        self.sensor_array.reset()?;
                    }
                }
            }

            let timer = Instant::now();
            self.sensor_array.read(&mut tmp)?;

            self.server.0.send(&tmp)?;

            delta = timer.elapsed();

            let tmp = self.frequency.checked_sub(delta).unwrap_or(Duration::ZERO);
            if tmp > Duration::ZERO {
                thread::sleep(tmp);
            }
        }
    }
}
