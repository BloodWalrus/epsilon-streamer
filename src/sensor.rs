use std::{
    error::Error,
    sync::{
        mpsc::{channel, Receiver, Sender, TryRecvError},
        Arc, Barrier, Condvar, Mutex, MutexGuard,
    },
    thread::{sleep, spawn, JoinHandle},
    time::{Duration, Instant},
};

use glam::{EulerRot::XYZ, Quat, Vec3A};
use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::{Mpu6050, Mpu6050Builder};

use crate::config::Device;

pub trait FromDevice {
    fn from_device(device: Device) -> Result<Self, Box<dyn Error>>
    where
        Self: Sized;
}

impl FromDevice for Mpu6050<I2cdev> {
    fn from_device(device: Device) -> Result<Self, Box<dyn Error>>
    where
        Self: Sized,
    {
        let mut delay = Delay;
        let mut mpu6050 = Mpu6050Builder::new()
            .i2c(I2cdev::new(format!("/dev/i2c-{}", device.bus))?)
            .slave_addr(device.addr)
            .build()?;

        mpu6050.init(&mut delay)?;

        Ok(mpu6050)
    }
}

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
        let mut delta = 0.0;

        let mut timer;
        loop {
            timer = Instant::now();

            self.rotation += self.device.get_gyro().unwrap() * delta;

            // if something can be read from notifier write rotation as quat into the output
            if let Ok(_) = self.notifier.try_recv() {
                let rot = &self.rotation;
                self.output
                    .send(Quat::from_euler(XYZ, rot.x, rot.y, rot.z))
                    .unwrap();

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

        for i in 0..N {
            // (output, outputs[i]) = channel();
            // (notifiers[i], notifier) = channel();

            let (tmp0, tmp1) = channel();
            let output = tmp0;
            unsafe {
                std::ptr::write(&mut outputs[i] as *mut Receiver<Quat>, tmp1);
            }

            let (tmp0, tmp1) = channel();
            let notifier = tmp1;
            unsafe {
                std::ptr::write(&mut notifiers[i] as *mut Sender<()>, tmp0);
            }

            // tmp[i] = (output, notifie(output, notifierr);

            unsafe {
                std::ptr::write(
                    &mut tmp[i] as *mut (Sender<Quat>, Receiver<()>),
                    (output, notifier),
                );
            }
        }

        // create sensors
        let devices = devices
            .into_iter()
            .zip(tmp.into_iter())
            .map(|(device, io)| {
                let (output, notifier) = io;
                Sensor::new(device, output, notifier, barrier.clone())
            })
            .enumerate();

        // start sensors
        for (i, sensor) in devices {
            // this unwrap call should never panic as the name should never contain null bytes
            // sensors[i] = std::thread::Builder::new()
            //     .name(format!("sensor {}", i))
            //     .spawn(move || sensor.main())
            //     .unwrap();

            unsafe {
                std::ptr::write(
                    &mut sensors[i] as *mut JoinHandle<()>,
                    std::thread::Builder::new()
                        .name(format!("sensor {}", i))
                        .spawn(move || sensor.main())
                        .unwrap(),
                )
            }
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
            notifier.send(()).unwrap();
        }

        for (i, output) in self.outputs.iter().enumerate() {
            dest[i] = output.recv().unwrap();
        }

        // synchronise all threads before continuing execution
        // this call should not block
        self.barrier.wait();
    }
}
