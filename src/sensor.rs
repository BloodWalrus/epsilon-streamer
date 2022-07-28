use std::{
    sync::{
        mpsc::{channel, Receiver, Sender, TryRecvError},
        Arc, Barrier, Condvar, Mutex, MutexGuard,
    },
    thread::{sleep, spawn, JoinHandle},
    time::{Duration, Instant},
};

use glam::{EulerRot::XYZ, Quat, Vec3A};
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;

// i hope all this works

// the sensors need a method communication to send shutdown signals or stop them from running when not in use
// it is currently not an issue so will do later

const DEFAULT_SENSOR_ROTATION: Vec3A = Vec3A::ZERO;

pub struct Sensor {
    device: Mpu6050<I2cdev>,
    output: Sender<Quat>,
    notifier: Receiver<()>,
    barrier: Arc<Barrier>,
    rotation: Vec3A,
}

impl Sensor {
    pub fn new(
        device: Mpu6050<I2cdev>,
        output: Sender<Quat>,
        notifier: Receiver<()>,
        barrier: Arc<Barrier>,
    ) -> Self {
        Self {
            device,
            output,
            notifier,
            barrier,
            rotation: DEFAULT_SENSOR_ROTATION,
        }
    }

    pub fn main(mut self) {
        let delta = 0.0;

        let mut timer;
        loop {
            timer = Instant::now();

            self.rotation += self.device.get_gyro().expect(todo!()) * delta;

            // if something can be read from notifier write rotation as quat into the output
            if let Ok(_) = self.notifier.try_recv() {
                let rot = &self.rotation;
                self.output
                    .send(Quat::from_euler(XYZ, rot.x, rot.y, rot.z))
                    .expect(todo!());

                // wait for all sensor threads to synchronise
                // this call should block for a short period of time
                self.barrier.wait();
            }

            delta = timer.elapsed().as_secs_f32();
        }
    }
}

pub struct SensorArray<const N: usize> {
    sensors: [JoinHandle<()>; N],
    outputs: [Receiver<Quat>; N],
    notifiers: [Sender<()>; N],
    barrier: Arc<Barrier>,
}

impl<const N: usize> SensorArray<N> {
    pub fn new(devices: [Mpu6050<I2cdev>; N]) -> Self {
        // create empty arrays
        let mut sensors: [JoinHandle<()>; N] = unsafe { std::mem::zeroed() };

        let mut outputs: [Receiver<Quat>; N] = unsafe { std::mem::zeroed() };
        let mut notifiers: [Sender<()>; N] = unsafe { std::mem::zeroed() };

        let mut tmp: [(Sender<Quat>, Receiver<()>); N] = unsafe { std::mem::zeroed() };

        // create barrier
        let barrier = Arc::new(Barrier::new(N + 1));

        // generate channels
        for i in 0..N {
            let output;
            let notifier;
            (output, outputs[i]) = channel();
            (notifiers[i], notifier) = channel();

            tmp[i] = (output, notifier);
        }

        // create sensors
        let devices = devices
            .into_iter()
            .zip(tmp.into_iter())
            .map(|(device, (output, notifier))| {
                Sensor::new(device, output, notifier, barrier.clone())
            })
            .enumerate();

        // start sensors
        for (i, sensor) in devices {
            // this unwrap call should never panic as the name should never contain null bytes
            sensors[i] = std::thread::Builder::new()
                .name(format!("sensor {}", i))
                .spawn(move || sensor.main())
                .unwrap()
        }

        Self {
            sensors,
            outputs,
            notifiers,
            barrier,
        }
    }

    // reads the sensors and returns their quaterion rotations
    pub fn read(&self, dest: &mut [Quat; N]) {
        for notifier in &self.notifiers {
            notifier.send(()).expect(todo!());
        }

        for (i, output) in self.outputs.iter().enumerate() {
            dest[i] = output.recv().expect(todo!());
        }

        // synchronise all threads before continuing execution
        // this call should not block
        self.barrier.wait();
    }
}
