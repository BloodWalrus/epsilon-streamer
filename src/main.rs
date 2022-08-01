use std::error::Error;

use streamer::Streamer;

mod config;
mod connection_listner;
mod sensor;
mod streamer;

const SENSOR_COUNT: usize = 7;

fn main() -> Result<(), Box<dyn Error>> {
    Streamer::<SENSOR_COUNT>::init()?.main()?;

    Ok(())
}
