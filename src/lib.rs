use map_3d::{Ellipsoid, geodetic2ecef};
use nalgebra::Vector3;

use crate::data_walker::DataWalker;
use crate::filter::Filter;
use crate::message::Message;
use crate::state::State;

mod message;
mod time;
mod data_walker;
pub mod state;
mod filter;

pub const ELLIPSOID: Ellipsoid = Ellipsoid::WGS84;

pub fn process_file(file_data: Vec<u8>) -> Vec<State> {
    let messages = preprocess(file_data);
    let mut filter = Filter::new();
    let mut result = vec![];
    let mut counter = 0usize;

    for message in messages {
        match message {
            Message::Imu { acceleration, gyro, .. } => {
                filter.step_imu(acceleration, gyro);
                if counter == 0 {
                    result.push(filter.get_state());
                }
                counter = (counter + 1) % 200;
            },
            Message::Gps { latitude, longitude, altitude, .. } => {
                let (x, y, z) = geodetic2ecef(latitude, longitude, altitude, ELLIPSOID);
                filter.observe_gps(Vector3::new(x, y, z));
            }
            _ => {}
        }
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
