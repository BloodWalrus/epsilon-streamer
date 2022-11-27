use crate::ahrs::Ahrs;
use std::{
    error::Error,
    marker::PhantomData,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc, Barrier,
    },
    thread::JoinHandle,
    time::Instant,
};

use glam::{EulerRot::XYZ, Quat, Vec3A};
use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::{Mpu6050, Mpu6050Builder};

use crate::config::Device;

use crate::Gyro;

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

// the sensors need a method communication to send shutdown signals or stop them from running when not in use
// it is currently not an issue so will do later

const SENSOR_FREQUENCY: f32 = 200.0;
const DEFAULT_SENSOR_ROTATION: Vec3A = Vec3A::ZERO;

pub struct Sensor {
    device: Gyro,
    output: Sender<Quat>,
    notifier: Receiver<()>,
    barrier: Arc<Barrier>,
    rotation: Vec3A,
}

impl Sensor {
    pub fn new(
        device: Gyro,
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

    pub fn calibrate(&mut self) {
        const SAMPLE_COUNT: usize = 1000;
        let mut tmp_gyro = Vec3A::ZERO;
        let mut tmp_acc = Vec3A::ZERO;
        for _ in 0..SAMPLE_COUNT {
            tmp_gyro += self.device.get_gyro().unwrap_or(Vec3A::ZERO);
            tmp_acc += self.device.get_acc().unwrap_or(Vec3A::ZERO);
        }
        tmp_gyro = (1.0 / SAMPLE_COUNT as f32) * tmp_gyro;
        tmp_acc = (1.0 / SAMPLE_COUNT as f32) * tmp_acc;
        self.device.gyro_offset = -tmp_gyro;
        self.device.acc_offset = -tmp_acc;
    }

    pub fn reset(&mut self) {
        self.rotation = Vec3A::ZERO;
    }

    pub fn main(mut self) {
        self.calibrate();
        let mut delta = 0.0;
        let round = |vec: Vec3A| (vec * 1000.0).round() / 1000.0;
        let mut timer;

        let mut ahrs = Ahrs::new(SENSOR_FREQUENCY);

        loop {
            timer = Instant::now();

            //self.rotation +=
            //    round(self.device.get_gyro().unwrap_or(Vec3A::ZERO)) * SENSOR_FREQUENCY.recip();
            ahrs.update_imu(
                self.device.get_gyro().unwrap_or(Vec3A::ZERO),
                self.device.get_acc().unwrap_or(Vec3A::ZERO),
            );

            // if something can be read from notifier write rotation as quat into the output
            if let Ok(_) = self.notifier.try_recv() {
                let rot = &self.rotation;
                self.output.send(ahrs.rotation()).unwrap();

                // wait for all sensor threads to synchronise
                // this call should block for a short period of time
                self.barrier.wait();
            }

            delta = timer.elapsed().as_secs_f32();
            if SENSOR_FREQUENCY - delta > 0.0 {
                std::thread::sleep(Duration::from_secs_f32(SENSOR_FREQUENCY - delta));
            }
        }
    }
}

pub struct SensorArray<const N: usize> {
    sensors: [JoinHandle<()>; N],
    outputs: [Receiver<Quat>; N],
    notifiers: [Sender<()>; N],
    barrier: Arc<Barrier>,
    _marker: PhantomData<Gyro>,
}

impl<const N: usize> SensorArray<N> {
    pub fn new(devices: [Gyro; N]) -> Self {
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
            _marker: PhantomData,
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
