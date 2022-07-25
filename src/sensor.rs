use std::{
    sync::{
        mpsc::{channel, Receiver, Sender, TryRecvError},
        Arc, Condvar, Mutex, MutexGuard,
    },
    thread::{sleep, spawn, JoinHandle},
    time::{Duration, Instant},
};

use glam::{EulerRot::XYZ, Quat, Vec3A};
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;

//##################
//add barrier to sensor, so that sensors can be synced
//##################

// i hope all this works

const DEFAULT_SENSOR_ROTATION: Vec3A = Vec3A::ZERO;

pub struct Sensor {
    device: Mpu6050<I2cdev>,
    output: Sender<Quat>,
    notifier: Receiver<()>,
    rotation: Vec3A,
}

impl Sensor {
    pub fn new(device: Mpu6050<I2cdev>, output: Sender<Quat>, notifier: Receiver<()>) -> Self {
        Self {
            device,
            output,
            notifier,
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
            }

            delta = timer.elapsed().as_secs_f32();
        }
    }
}

pub struct SensorArray<const N: usize> {
    sensors: [JoinHandle<()>; N],
    outputs: [Receiver<Quat>; N],
    notifiers: [Sender<()>; N],
}

impl<const N: usize> SensorArray<N> {
    pub fn new(devices: [Mpu6050<I2cdev>; N]) -> Self {
        // create empty arrays
        let mut sensors: [JoinHandle<()>; N] = unsafe { std::mem::zeroed() };

        let mut outputs: [Receiver<Quat>; N] = unsafe { std::mem::zeroed() };
        let mut notifiers: [Sender<()>; N] = unsafe { std::mem::zeroed() };

        let mut tmp: [(Sender<Quat>, Receiver<()>); N] = unsafe { std::mem::zeroed() };

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
            .map(|(device, (output, notifier))| Sensor::new(device, output, notifier))
            .enumerate();

        // start sensors
        for (i, sensor) in devices {
            sensors[i] = std::thread::spawn(move || sensor.main())
        }

        Self {
            sensors,
            outputs,
            notifiers,
        }
    }

    // reads the sensors and returns there quaterion rotations
    pub fn read(&self) -> Box<[Quat; N]> {
        let mut tmp = Box::new([Quat::IDENTITY; N]);

        for notifier in &self.notifiers {
            notifier.send(()).expect(todo!());
        }

        for (i, output) in self.outputs.iter().enumerate() {
            tmp[i] = output.recv().expect(todo!());
        }

        // use barrier to ensure all sensors begin updating again at the same time.

        tmp
    }
}
