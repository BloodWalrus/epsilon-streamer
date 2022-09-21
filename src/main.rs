use std::error::Error;

use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050;
use streamer::Streamer;

mod config;
mod connection_listner;
mod sensor;
mod streamer;

const SENSOR_COUNT: usize = 7;
type GYRO = Mpu6050<I2cdev>;

fn main() -> Result<(), Box<dyn Error>> {
    Streamer::<SENSOR_COUNT, GYRO>::new()?.main()?;

    Ok(())
}
