use std::time::Duration;
use ::time::{Date, Month};
use map_3d::{Ellipsoid, geodetic2ecef};
use nalgebra::Vector3;

use crate::data_walker::DataWalker;
// use crate::filter::Filter;
use crate::message::Message;
use crate::state::State;

mod message;
mod time;
mod data_walker;
pub mod state;
// mod filter;
// mod jacobian;

// pub const ELLIPSOID: Ellipsoid = Ellipsoid::WGS84;

pub const STEP_TIME: Duration = Duration::from_millis(5);

pub const INITIAL_ACCEL_FIX: f64 = 1.0 / (200.0 * 60.0 * 10.0); // 1 per 10 minutes
pub const INITIAL_ACCEL_UPDATE: f64 = 1.0 / (200.0 * 1.0); // 1 per 1 second
pub const STABLE_ACCEL_FIX: f64 = 1.0 / (200.0); // 1 per 1 second
pub const STABLE_ACCEL_UPDATE: f64 = 1.0 / (200.0 * 60.0 * 2.0); // 1 per 2 minutes

pub const INITIAL_MAGNETO_FIX: f64 = 1.0 / (60.0 * 30.0); // 1 per 30 minutes
pub const INITIAL_MAGNETO_UPDATE: f64 = 1.0 / (3.0); // 1 per 3 seconds
pub const STABLE_MAGNETO_FIX: f64 = 1.0 / (15.0); // 1 per 15 seconds
pub const STABLE_MAGNETO_UPDATE: f64 = 1.0 / (60.0 * 6.0); // 1 per 6 minutes

const fn accel_fix(count: usize) -> f64 {
    if count < 2000 {
        INITIAL_ACCEL_FIX
    } else {
        STABLE_ACCEL_FIX
    }
}
fn accel_update(count: usize) -> f64 {
    if count < 2000 {
        1.0 / ((count + 1) as f64)
    } else {
        STABLE_ACCEL_UPDATE
    }
}
const fn magneto_fix(count: usize) -> f64 {
    if count < 2000 {
        INITIAL_MAGNETO_FIX
    } else {
        STABLE_MAGNETO_FIX
    }
}
const fn magneto_update(count: usize) -> f64 {
    if count < 2000 {
        INITIAL_MAGNETO_UPDATE
    } else {
        STABLE_MAGNETO_UPDATE
    }
}

pub fn process_file(file_data: Vec<u8>) -> Vec<State> {
    let messages = preprocess(file_data);
    // let mut filter = Filter::new();
    let mut state = State::new();
    let mut result = vec![];
    let mut counter = 0usize;
    // let mut initialized = false;

    for message in messages {
        match message {
            Message::Imu { acceleration, gyro, .. } => {
                state.update_gyro(gyro, STEP_TIME);
                state.update_accel(acceleration, accel_fix(result.len()), accel_update(result.len()));
                // filter.step_imu(acceleration, gyro);
                if counter == 0 {
                    result.push(state.clone());
                }
                counter = (counter + 1) % 200;
            },
            Message::Gps { latitude, longitude, altitude, .. } => {
                // let (x, y, z) = geodetic2ecef(latitude, longitude, altitude, ELLIPSOID);
                // if !initialized {
                //     initialized = true;
                //     filter.set_position(Vector3::new(x, y, z));
                // }
                // filter.observe_gps(Vector3::new(x, y, z));
            }
            Message::Slow { magnetic, .. } => {
                state.update_magneto(magnetic, magneto_fix(result.len()), magneto_update(result.len()));
                // filter.observe_magnetometer(magnetic, Date::from_calendar_date(2024, Month::April, 8).unwrap());
            }
        }

        // if result.len() >= 10000 {
        //     break
        // }
    }

    return result;
}

fn preprocess(file_data: Vec<u8>) -> Vec<Message> {
    let mut result = vec![];
    let mut file_data = DataWalker::new(file_data);
    let mut real_data_started = false;

    let mut last_altitude = 0.0;
    let mut gps_since_last_altitude = vec![];

    loop {
        let Ok(message) = (&mut file_data).try_into() else { break; };
        if !real_data_started {
            if let Message::Gps { fixquality, altitude, .. } = message {
                if fixquality != 0 {
                    real_data_started = true;
                    result.push(message);
                    last_altitude = altitude;
                }
            }
            continue;
        }
        match message {
            Message::Gps { altitude, .. } => {
                if altitude == last_altitude {
                    gps_since_last_altitude.push(result.len());
                } else {
                    for gps_index in 0..gps_since_last_altitude.len() {
                        let message_index = gps_since_last_altitude[gps_index];
                        let computed_altitude = last_altitude + ((gps_index + 1) as f64) * (altitude - last_altitude) / ((gps_since_last_altitude.len() + 1) as f64);
                        let Message::Gps { ref mut altitude, .. } = &mut result[message_index] else { panic!("This shouldn't happen") };
                        *altitude = computed_altitude;
                    }
                    gps_since_last_altitude.clear();
                    last_altitude = altitude;
                }
            }
            _ => { } // Intentionally Left Blank
        }
        result.push(message);
    }

    result
}
