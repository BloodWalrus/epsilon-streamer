use std::mem::size_of;

use ecore::EpsilonResult;
use glam::Quat;
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;
use streamer::EpsilonStreamer;

mod ahrs;
mod config;
mod sensor;
mod streamer;

const SENSOR_COUNT: usize = 7;
const QUAT_ARRAY_SIZE: usize = size_of::<[Quat; SENSOR_COUNT]>();
type Gyro = Mpu6050<I2cdev>;

fn main() -> EpsilonResult<()> {
    EpsilonStreamer::new()?.run()?;

    Ok(())
}
