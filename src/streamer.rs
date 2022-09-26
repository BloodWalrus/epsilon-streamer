// all this code needs cleaning up

use std::{
    collections::VecDeque,
    error::Error,
    fmt::Display,
    net::SocketAddr,
    sync::{Arc, Mutex, RwLock},
    thread::JoinHandle,
};

use ecore::connection::Connection;
use glam::Quat;
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;

use crate::{
    config::{Config, ConfigError},
    connection_listner::ConnectionListner,
    sensor::{FromDevice, Gyro, SensorArray},
    GYRO, SENSOR_COUNT,
};

const CONFIG_PATH: &str = "./config.toml";

#[derive(Debug)]
pub enum StreamerError {
    ConfigInvalid(ConfigError),
}

impl Display for StreamerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let tmp;
        f.write_str(match self {
            StreamerError::ConfigInvalid(config_error) => {
                tmp = format!("config error: {}", config_error);
                &tmp
            }
        })
    }
}

impl Error for StreamerError {}

pub struct Streamer {
    sensor_array: SensorArray<SENSOR_COUNT>,
    connection_listner: Arc<Mutex<ConnectionListner<SENSOR_COUNT, Quat>>>,
    unhandled_connections: Arc<Mutex<VecDeque<Connection<SENSOR_COUNT, Quat>>>>,
    streams: Vec<JoinHandle<()>>,
}

impl Streamer {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        // load bytes from config
        let config = std::fs::read(CONFIG_PATH)?;
        // parse config
        let config: Config = toml::from_slice(&config)?;

        // check the config device count is correct
        if config.devices.len() != SENSOR_COUNT {
            if config.devices.len() > SENSOR_COUNT {
                Err(StreamerError::ConfigInvalid(ConfigError::TooManyDevices))?
            } else {
                Err(StreamerError::ConfigInvalid(ConfigError::NotEnoughDevices))?
            }
        }

        // attempt to create a connection listner from the port and ip address in the config
        // this will only bind one connection listner due to iterators lazziness
        let connection_listner = config
            .ip_addresses
            .iter()
            .map(|ip| SocketAddr::new(*ip, config.port))
            .map(|socket| ConnectionListner::bind(socket))
            .filter_map(|listner| match listner {
                Ok(listner) => Some(listner),
                Err(_) => None,
            })
            .nth(0);

        // return error if no connection_listner could be bound
        let connection_listner = match connection_listner {
            Some(listner) => listner,
            None => Err(StreamerError::ConfigInvalid(
                ConfigError::NoValidSocketAddrs,
            ))?,
        };

        let mut devices: [GYRO; SENSOR_COUNT] = unsafe { std::mem::zeroed() };
        for (i, device) in config
            .devices
            .into_iter()
            .map(|device| GYRO::from_device(device))
            .enumerate()
        {
            devices[i] = device?;
        }

        let sensor_array = SensorArray::new(devices);

        Ok(Self {
            sensor_array,
            connection_listner: Arc::new(Mutex::new(connection_listner)),
            streams: Vec::new(),
            unhandled_connections: Arc::new(Mutex::new(VecDeque::new())),
        })
    }

    pub fn main(mut self) -> Result<(), Box<dyn Error>> {
        // clone references pass to the connection listner closure
        let unhandled_connections = self.unhandled_connections.clone();
        let connection_listner = self.connection_listner.clone();

        // create connection listner and start listning for connections
        let connection_listner = std::thread::Builder::new()
            .name("efbt connection listner".into())
            .spawn(move || Self::connection_listner(unhandled_connections, connection_listner))?;

        let sensor_data = Arc::new(RwLock::new(Box::new([Quat::IDENTITY; SENSOR_COUNT])));

        loop {
            if connection_listner.is_finished() {
                break;
            }

            for connection in self.unhandled_connections.lock().unwrap().drain(..) {
                let addr = connection.peer_addr()?;
                let stream = Stream::new(connection, sensor_data.clone());

                // start stream
                self.streams.push(
                    std::thread::Builder::new()
                        .name(format!("stream on {}", addr))
                        .spawn(move || stream.main())?,
                );
            }

            // reading sensors and storing result for later so that all streams do not need to poll sensor
            self.sensor_array.read(&mut **sensor_data.write().unwrap());

            // unpark threads so that streams will send sensor data to the connected server
            // the streams are really here so that data can be send on each connection in "parrallel" as it should be a bit faster
            for stream in &self.streams {
                stream.thread().unpark();
            }
        }

        connection_listner.join();

        Ok(())
    }

    pub fn connection_listner(
        unhandled_connections: Arc<Mutex<VecDeque<Connection<SENSOR_COUNT, Quat>>>>,
        connection_listner: Arc<Mutex<ConnectionListner<SENSOR_COUNT, Quat>>>,
    ) {
        for connection in connection_listner
            .lock()
            .expect("mutex poisoned")
            .incomming()
        {
            unhandled_connections.lock().unwrap().push_back(connection)
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct Stream<const SENSOR_COUNT: usize> {
    connection: Connection<SENSOR_COUNT, Quat>,
    sensor_data: Arc<RwLock<Box<[Quat; SENSOR_COUNT]>>>,
}

impl<const SENSOR_COUNT: usize> Stream<SENSOR_COUNT> {
    pub fn new(
        connection: Connection<SENSOR_COUNT, Quat>,
        sensor_data: Arc<RwLock<Box<[Quat; SENSOR_COUNT]>>>,
    ) -> Self {
        Self {
            connection,
            sensor_data,
        }
    }

    pub fn main(mut self) {
        loop {
            std::thread::park();

            let result = self.connection.send(
                &**self
                    .sensor_data
                    .read()
                    .expect("rwlock on stream thread failed"),
            );

            if let Err(_) = result {
                break;
            }
            // .expect("failed to send frame");
        }
    }
}
