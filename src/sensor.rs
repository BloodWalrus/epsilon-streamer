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

use ecore::EpsilonResult;
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

struct Sensor {
    device: Gyro,
    output: Sender<Quat>,
    messages: Receiver<Msg>,
    barrier: Arc<Barrier>,
    rotation: Vec3A,
    starter: Arc<Barrier>,
}

impl Sensor {
    pub fn new(
        device: Gyro,
        output: Sender<Quat>,
        messages: Receiver<Msg>,
        barrier: Arc<Barrier>,
        starter: Arc<Barrier>,
    ) -> Self {
        Self {
            device,
            output,
            messages,
            barrier,
            starter,
            rotation: Vec3A::ZERO,
        }
    }

    pub fn calibrate(&mut self) {
        const SAMPLE_COUNT: usize = 1000;
        self.device.gyro_offset = Vec3A::ZERO;
        let mut tmp = Vec3A::ZERO;
        for _ in 0..SAMPLE_COUNT {
            tmp += self.device.get_gyro().unwrap_or(Vec3A::ZERO);
        }
        tmp = (1.0 / SAMPLE_COUNT as f32) * tmp;
        self.device.gyro_offset = -tmp;
    }

    pub fn reset(&mut self) {
        self.rotation = Vec3A::ZERO;
    }

    pub fn main(mut self) {
        let mut delta = 0.0;
        let round = |vec: Vec3A| (vec * 100.0).round() / 100.0;
        let mut timer;

        self.starter.wait();
        self.calibrate();
        loop {
            timer = Instant::now();

            self.rotation += round(self.device.get_gyro().unwrap_or(Vec3A::ZERO)) * delta;

            match self.messages.try_recv() {
                Ok(msg) => match msg {
                    Msg::Reset => {
                        delta = 0.0;
                        self.calibrate();
                        self.reset();
                        self.barrier.wait();
                        continue;
                    }
                    Msg::Read => {
                        let rot = round(self.rotation);
                        self.output
                            .send(Quat::from_euler(XYZ, rot.x, rot.y, rot.z))
                            .unwrap();

                        // wait for all sensor threads to synchronise
                        // this call should block for a short period of time
                        self.barrier.wait();
                    }
                },
                Err(err) => match err {
                    std::sync::mpsc::TryRecvError::Empty => (),
                    std::sync::mpsc::TryRecvError::Disconnected => {
                        panic!("message bus for sensor diconnect prematurely.")
                    }
                },
            }

            delta = timer.elapsed().as_secs_f32();
        }
    }
}

#[repr(u8)]
enum Msg {
    Reset,
    Read,
}

pub struct SensorArray<const N: usize> {
    sensors: [JoinHandle<()>; N],
    outputs: [Receiver<Quat>; N],
    messages: [Sender<Msg>; N],
    barrier: Arc<Barrier>,
    starter: Arc<Barrier>,
    _marker: PhantomData<Gyro>,
}

impl<const N: usize> SensorArray<N> {
    pub fn new(devices: [Gyro; N]) -> Self {
        // create empty arrays
        let mut sensors: [JoinHandle<()>; N] = unsafe { std::mem::zeroed() };

        let mut outputs: [Receiver<Quat>; N] = unsafe { std::mem::zeroed() };
        let mut messages: [Sender<Msg>; N] = unsafe { std::mem::zeroed() };

        let mut tmp: [(Sender<Quat>, Receiver<Msg>); N] = unsafe { std::mem::zeroed() };

        // create barrier
        let barrier = Arc::new(Barrier::new(N + 1));
        let starter = Arc::new(Barrier::new(N + 1));

        for i in 0..N {
            let (tmp0, tmp1) = channel();
            let output = tmp0;
            unsafe {
                std::ptr::write(&mut outputs[i] as *mut Receiver<Quat>, tmp1);
            }

            let (tmp0, tmp1) = channel();
            let notifier = tmp1;
            unsafe {
                std::ptr::write(&mut messages[i] as *mut Sender<Msg>, tmp0);
            }

            unsafe {
                std::ptr::write(
                    &mut tmp[i] as *mut (Sender<Quat>, Receiver<Msg>),
                    (output, notifier),
                );
            }
        }

        // create sensors
        let devices = devices
            .into_iter()
            .zip(tmp.into_iter())
            .map(|(device, io)| {
                let (output, messages) = io;
                Sensor::new(device, output, messages, barrier.clone(), starter.clone())
            })
            .enumerate();

        // start sensors
        for (i, sensor) in devices {
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
            messages,
            barrier,
            starter,
            _marker: PhantomData,
        }
    }

    // reads the sensors and returns their quaterion rotations
    pub fn read(&self, dest: &mut [Quat; N]) -> EpsilonResult<()> {
        for messages in &self.messages {
            messages.send(Msg::Read)?;
        }

        for (i, output) in self.outputs.iter().enumerate() {
            dest[i] = output.recv()?;
        }

        // synchronise all threads before continuing execution
        // this call should not block
        self.barrier.wait();
        Ok(())
    }

    pub fn start(&self) {
        self.starter.wait();
    }

    pub fn reset(&self) -> EpsilonResult<()> {
        for messages in &self.messages {
            messages.send(Msg::Reset)?;
        }

        self.barrier.wait();
        Ok(())
    }
}
