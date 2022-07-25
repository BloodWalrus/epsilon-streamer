use std::{
    sync::{
        mpsc::{channel, Sender, TryRecvError},
        Arc, Mutex,
    },
    thread::{sleep, spawn, JoinHandle},
    time::{Duration, Instant},
};

use glam::Vec3A;
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;

pub struct Sensor {
    rotation: Arc<Mutex<Vec3A>>,
    worker: Option<JoinHandle<()>>,
    worker_msg_bus: Sender<()>,
}

// i hope this works

impl Sensor {
    pub fn new(
        mut device: Mpu6050<I2cdev>,
        min_delta: Duration,
        update_frequency: Duration,
    ) -> Self {
        let rotation = Arc::new(Mutex::new(Vec3A::ZERO));
        let worker_rotation = rotation.clone();
        let (tx, rx) = channel();

        // creates a thread to update the sensor
        let worker = Some(spawn(move || {
            let should_shutdown = || rx.try_recv().is_ok();

            let mut rotation = Vec3A::ZERO;

            let mut update_delta = Duration::from_millis(0);
            let mut delta = Instant::now();

            loop {
                if !should_shutdown() {
                    let vel = device.get_gyro().expect("failed to read from device.");
                    // Sensor.rotation += angular_velocity * delta_time
                    rotation += vel * delta.elapsed().as_secs_f32();

                    sleep(min_delta);
                    update_delta += delta.elapsed();
                    delta = Instant::now();

                    if update_delta >= update_frequency {
                        *worker_rotation.lock().expect("Mutex poisoned") = rotation;
                        update_delta = Duration::from_millis(0);
                    }
                }
            }
        }));

        Self {
            rotation,
            worker,
            worker_msg_bus: tx,
        }
    }

    pub fn get_rotation(&self) -> Vec3A {
        self.rotation.lock().expect("Mutex poisoned").clone()
    }

    pub fn shutdown(&mut self) {
        self.worker_msg_bus.send(()).unwrap();
        let _ = self.worker.take().unwrap();
        self.worker = None;
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        self.shutdown();
    }
}
