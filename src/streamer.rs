// all this code needs cleaning up

use std::{
    collections::VecDeque,
    error::Error,
    sync::{Arc, Mutex, RwLock},
    thread::JoinHandle,
};

use ecore::connection::Connection;
use glam::Quat;

use crate::{connection_listner::ConnectionListner, sensor::SensorArray};

pub struct Streamer<const SENSOR_COUNT: usize> {
    sensor_array: SensorArray<SENSOR_COUNT>,
    connection_listner: Arc<Mutex<ConnectionListner<SENSOR_COUNT, Quat>>>,
    unhandled_connections: Arc<RwLock<VecDeque<Connection<SENSOR_COUNT, Quat>>>>,
    streams: Vec<JoinHandle<()>>,
}

impl<const SENSOR_COUNT: usize> Streamer<SENSOR_COUNT> {
    pub fn init() -> Result<Self, Box<dyn Error>> {
        // read ~/.config/efbt/config.toml

        Ok(Self {
            sensor_array: todo!(),
            connection_listner: todo!(),
            streams: todo!(),
            unhandled_connections: todo!(),
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

            for connection in self.unhandled_connections.write().unwrap().drain(..) {
                let addr = connection.peer_addr().unwrap();
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

        todo!()
    }

    pub fn connection_listner(
        unhandled_connections: Arc<RwLock<VecDeque<Connection<SENSOR_COUNT, Quat>>>>,
        connection_listner: Arc<Mutex<ConnectionListner<SENSOR_COUNT, Quat>>>,
    ) {
        for connection in connection_listner
            .lock()
            .expect("mutex poisoned")
            .incomming()
        {
            unhandled_connections.write().unwrap().push_back(connection)
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

            self.connection
                .send(
                    &**self
                        .sensor_data
                        .read()
                        .expect("rwlock on stream thread failed"),
                )
                .expect("failed to send frame");
        }
    }
}
