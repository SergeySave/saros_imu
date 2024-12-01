use std::f64::consts::PI;
use std::time::Duration;
use ::time::{Date, Month};
use map_3d::{Ellipsoid, geodetic2ecef, geodetic2enu};
use nalgebra::Vector3;

use crate::data_walker::DataWalker;
// use crate::filter::Filter;
use crate::message::Message;
use crate::state::{ExtraOutput, State};

mod message;
mod time;
mod data_walker;
pub mod state;
// mod filter;
// mod jacobian;

pub const ELLIPSOID: Ellipsoid = Ellipsoid::WGS84;

pub const STEP_TIME: Duration = Duration::from_millis(5);

pub const INITIAL_ACCEL_FIX: f64 = 1.0 / (200.0 * 60.0 * 10.0); // 1 per 10 minutes
pub const INITIAL_ACCEL_UPDATE: f64 = 1.0 / (200.0 * 1.0); // 1 per 1 second
pub const STABLE_ACCEL_FIX: f64 = 1.0 / (200.0 * 20.0); // 1 per 20 seconds
pub const STABLE_ACCEL_UPDATE: f64 = 1.0 / (200.0 * 60.0 * 10.0); // 1 per 10 minutes

pub const INITIAL_MAGNETO_FIX: f64 = 1.0 / (60.0 * 30.0); // 1 per 30 minutes
pub const INITIAL_MAGNETO_UPDATE: f64 = 1.0 / (3.0); // 1 per 3 seconds
pub const STABLE_MAGNETO_FIX: f64 = 1.0 / (20.0); // 1 per 20 seconds
pub const STABLE_MAGNETO_UPDATE: f64 = 1.0 / (60.0 * 10.0); // 1 per 10 minutes

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
fn magneto_update(count: usize) -> f64 {
    if count < 2000 {
        1.0 / ((count + 1) as f64)
    } else {
        STABLE_MAGNETO_UPDATE
    }
}

pub fn process_file(file_data: Vec<u8>) -> (Vec<State>, Vec<ExtraOutput>) {
    let messages = preprocess(file_data);
    // let mut filter = Filter::new();
    let mut state = State::new();
    let mut result = vec![];
    let mut extra = vec![];
    let mut counter = 0usize;
    let mut initialized = false;
    let mut initial_lat = 0.0;
    let mut initial_lon = 0.0;

    let mut last_time: Option<u64> = None;
    let mut next_output = ExtraOutput::default();

    for message in messages {
        match message {
            Message::Imu { acceleration, gyro, time, .. } => {
                // match last_time {
                //     None => {
                //         last_time = Some(1)
                //     }
                //     Some(time) => {
                //         extra.push(next_output);
                //         last_time = Some(time + 1);
                //     }
                // }
                // next_output = ExtraOutput {
                //     time: last_time.unwrap() as f64 / 200.0,
                //     acceleration_x: acceleration.x,
                //     acceleration_y: acceleration.y,
                //     acceleration_z: acceleration.z,
                //     gyro_x: gyro.x,
                //     gyro_y: gyro.y,
                //     gyro_z: gyro.z,
                //     ..ExtraOutput::default()
                // };
                state.update_gyro(gyro, STEP_TIME);
                state.update_accel(acceleration, accel_fix(result.len()), accel_update(result.len()));
                // filter.step_imu(acceleration, gyro);
                if counter == 0 {
                    result.push(state.cleanup());
                    // println!("{:?}", time);
                }
                counter = (counter + 1) % 200;
            },
            Message::Gps { latitude, longitude, altitude, speed, .. } => {
                // next_output.latitude = Some(latitude);
                // next_output.longitude = Some(longitude);
                // next_output.altitude = Some(altitude);
                // next_output.speed = Some(speed);
                // if last_time.is_none() {
                //     extra.push(next_output);
                //     next_output = ExtraOutput::default();
                // }
                // if !initialized {
                //     initialized = true;
                //     initial_lat = latitude;
                //     initial_lon = longitude;
                // }
                // let (e, n, _) = geodetic2enu(latitude, longitude, 0.0, initial_lat, initial_lon, 0.0, ELLIPSOID);
                // let (e, n, u) = geodetic2enu(latitude, longitude, altitude, initial_lat, initial_lon, 0.0, ELLIPSOID);
                // extra.push(ExtraOutput {
                //     time: result.len() as f64,
                //     n: e,
                //     e: n,
                //     alt: altitude,
                // });
            }
            Message::Slow { magnetic, /*temperature,
                pressure,
                analog_sensors,
                battery_voltage,
                heater_state,*/ .. } => {
                // next_output.magneto_x = Some(magnetic.x);
                // next_output.magneto_y = Some(magnetic.y);
                // next_output.magneto_z = Some(magnetic.z);
                state.update_magneto(magnetic, magneto_fix(result.len()), magneto_update(result.len()));
                // extra.push(ExtraOutput {
                //     index: result.len(),
                //     temperature,
                //     pressure,
                //     analog0: analog_sensors[0],
                //     analog1: analog_sensors[1],
                //     analog2: analog_sensors[2],
                //     analog3: analog_sensors[3],
                //     analog4: analog_sensors[4],
                //     analog5: analog_sensors[5],
                //     analog6: analog_sensors[6],
                //     analog7: analog_sensors[7],
                //     analog8: analog_sensors[8],
                //     analog9: analog_sensors[9],
                //     battery_voltage,
                //     heater_state,
                // });
                // filter.observe_magnetometer(magnetic, Date::from_calendar_date(2024, Month::April, 8).unwrap());
            }
        }

        // if result.len() >= 10000 {
        //     break
        // }
    }

    return (result, extra);
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
