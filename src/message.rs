use std::error::Error;
use std::f64::consts::PI;
use std::fmt::{Debug, Display, Formatter};
use nalgebra::Vector3;
use crate::data_walker::DataWalker;
use crate::time::Timestamp;

#[derive(Debug)]
pub enum Message {
    Gps {
        // hour: u8,
        // minute: u8,
        // seconds: u8,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        speed: f64,
        fixquality: u8,
        // satellites: u8,
    },
    Imu {
        time: Timestamp,
        acceleration: Vector3<f64>,
        gyro: Vector3<f64>,
    },
    Slow {
        time: Timestamp,
        magnetic: Vector3<f64>,
        // temperature: f32,
        // pressure: f32,
        // analog_sensors: [f32; 10],
        // battery_voltage: f32,
        // heater_state: bool,
    }
}

#[derive(Debug)]
pub struct ParseError;

impl Display for ParseError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "ParseError")
    }
}

impl Error for ParseError {}

impl TryFrom<&mut DataWalker> for Message {
    type Error = ParseError;

    fn try_from(value: &mut DataWalker) -> Result<Self, Self::Error> {
        let type_identifier = value.read::<u8>().ok_or(ParseError)?;

        match type_identifier {
            0x10 => read_gps_message(value),
            0x20 => read_imu_message(value),
            0x30 => read_slow_message(value),
            x => {
                println!("{}", x);
                Err(ParseError)
            },
        }
    }
}

fn read_gps_message(value: &mut DataWalker) -> Result<Message, ParseError> {
    value.skip(3);
    // let hour = value.read::<u8>().ok_or_else(|| "Could not read gps hour".into())?;
    // let minute = value.read::<u8>().ok_or_else(|| "Could not read gps minute".into())?;
    // let seconds = value.read::<u8>().ok_or_else(|| "Could not read gps seconds".into())?;
    let latitude = value.read::<i32>().ok_or(ParseError)?;
    let longitude = value.read::<i32>().ok_or(ParseError)?;
    let altitude = value.read::<f32>().ok_or(ParseError)?;
    // value.skip(4);
    let speed = value.read::<f32>().ok_or(ParseError)?;
    let fixquality = value.read::<u8>().ok_or(ParseError)?;
    value.skip(1);
    // let satellites = value.read::<u8>().ok_or_else(|| "Could not read gps satellites".into())?;

    Ok(Message::Gps {
        latitude: (latitude as f64 / 10000000.0) * PI / 180.0,
        longitude: (longitude as f64 / 10000000.0) * PI / 180.0,
        altitude: altitude as f64,
        speed: speed as f64,
        fixquality,
    })
}

fn read_timestamp(value: &mut DataWalker) -> Result<Timestamp, ParseError> {
    let year = value.read::<u8>().ok_or(ParseError)?;
    let month = value.read::<u8>().ok_or(ParseError)?;
    let day = value.read::<u8>().ok_or(ParseError)?;
    let hour = value.read::<u8>().ok_or(ParseError)?;
    let minutes = value.read::<u8>().ok_or(ParseError)?;
    let seconds = value.read::<u8>().ok_or(ParseError)?;
    let milliseconds = value.read::<u16>().ok_or(ParseError)?;

    Ok(Timestamp {
        year,
        month,
        day,
        hour,
        minutes,
        seconds,
        milliseconds,
    })
}

fn read_imu_message(value: &mut DataWalker) -> Result<Message, ParseError> {
    let time = read_timestamp(value)?;
    let accel_x = value.read::<f32>().ok_or(ParseError)?;
    let accel_y = value.read::<f32>().ok_or(ParseError)?;
    let accel_z = -value.read::<f32>().ok_or(ParseError)?;
    let gyro_x = value.read::<f32>().ok_or(ParseError)?;
    let gyro_y = value.read::<f32>().ok_or(ParseError)?;
    let gyro_z = -value.read::<f32>().ok_or(ParseError)?;

    Ok(Message::Imu {
        time,
        acceleration: Vector3::new(accel_x as f64, accel_y as f64, accel_z as f64),
        gyro: Vector3::new(gyro_x as f64, gyro_y as f64, gyro_z as f64),
    })
}

fn read_slow_message(value: &mut DataWalker) -> Result<Message, ParseError> {
    let time = read_timestamp(value)?;
    let magnetic_x = -value.read::<f32>().ok_or(ParseError)?;
    let magnetic_y = value.read::<f32>().ok_or(ParseError)?;
    let magnetic_z = -value.read::<f32>().ok_or(ParseError)?;
    value.skip(4);
    value.skip(4);
    value.skip(4 * 10);
    value.skip(4);
    value.skip(1);

    Ok(Message::Slow {
        time,
        magnetic: Vector3::new(magnetic_x as f64, magnetic_y as f64, magnetic_z as f64),
    })
}

