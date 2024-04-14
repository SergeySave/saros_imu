use std::time::Duration;
use map_3d::{EARTH_RADIUS, ecef2geodetic, Ellipsoid, geodetic2ecef};
use nalgebra::{Matrix2, Point3, SVector, Vector2, Vector3};
use crate::data_walker::DataWalker;
use crate::filter::Filter;
use crate::message::Message;
use crate::state::State;
use crate::time::Timestamp;

mod message;
mod time;
mod data_walker;
mod state;
mod filter;

pub fn process_file(file_data: Vec<u8>) -> Vec<()> {
    // let mut filter = eskf::Builder::new()
    //     // .acceleration_variance(0.1)
    //     // .acceleration_bias(0.1)
    //     // .rotation_variance(0.1)
    //     // .rotation_bias(0.1)
    //     .initial_covariance(0.1)
    //     .build();
    // filter.gravity.z = 0.0;
    // let gps_variance = eskf::ESKF::variance_from_element(0.1);
    let mut filter = Filter::new();

    let mut file_data = DataWalker::new(file_data);
    let mut real_data_started = false;
    let mut x_offset: f64 = 0.0;
    let mut y_offset: f64 = 0.0;
    let mut z_offset: f64 = 0.0;

    loop {
        let Ok(x) = (&mut file_data).try_into() else { break; };
        if !real_data_started {
            if let Message::Gps { fixquality, latitude, longitude, altitude, .. } = x {
                if fixquality != 0 {
                    real_data_started = true;
                    (x_offset, y_offset, z_offset) = geodetic2ecef(latitude, longitude, altitude, Ellipsoid::WGS84);
                    // filter.observe_gps(Vector3::new(x_offset, y_offset, z_offset));
                    filter.set_position(Vector3::new(x_offset, y_offset, z_offset));
                }
            }
            continue;
        }
        // let mut before = filter.clone();
        match x {
            Message::Imu { acceleration, gyro, .. } => {
                filter.step_imu(acceleration, gyro);
                filter.debug();
                // filter.predict(
                //     acceleration,
                //     gyro,
                //     Duration::from_millis(5)
                // );
                // println!("{}{}{}", filter.position, filter.gravity, filter.gravity.magnitude());
                // println!("{}", filter.velocity_uncertainty());
                // println!("{}", filter.gravity);
                // println!("{:?}", filter.gravity);
                // println!("{:?}", filter.velocity);
                // println!("{:?}", filter.accel_bias.magnitude());
                // filter.accel_bias *= 0.95;
                // filter.rot_bias *= 0.95;
                // if filter.position.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN P");
                // }
                // if filter.velocity.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN V");
                // }
                // if filter.orientation.coords.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN O");
                // }
                // if filter.accel_bias.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN A");
                // }
                // if filter.rot_bias.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN R");
                // }
                // if filter.gravity.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN G");
                // }
                // if filter.position_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN PU");
                // }
                // if filter.velocity_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN VU");
                // }
                // if filter.orientation_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN OU");
                // }
            },
            Message::Gps { latitude, longitude, altitude, .. } => {
                // ecef2geodetic()
                let (x, y, z) = geodetic2ecef(latitude, longitude, altitude, Ellipsoid::WGS84);
                filter.observe_gps(Vector3::new(x_offset, y_offset, z_offset));
                let geodetic = ecef2geodetic(filter.get_state().position.x, filter.get_state().position.y, filter.get_state().position.z, Ellipsoid::WGS84);
                println!("{latitude} {longitude} {altitude}");
                println!("{:?}", geodetic);
                println!("{:?}", filter.get_state().velocity.magnitude());
                println!("{:?}", filter.get_state().gravity.magnitude());
                // filter.observe_position(
                //     Point3::new((x - x_offset), (y - y_offset), (z - z_offset)),
                //     gps_variance,
                // )
                //     .expect("Failed to observe GPS position");
                // println!("{:?}", filter.position);
                // if filter.position.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     println!("{:?}", filter);
                //     println!("{:?}", filter.velocity_uncertainty());
                //     println!("{:?}", before.velocity_uncertainty());
                //     before.observe_position(
                //         Point3::new((x - x_offset), (y - y_offset), (z - z_offset)),
                //         gps_variance,
                //     )
                //         .expect("Failed to observe GPS position");
                //     panic!("NaN P");
                // }
                // if filter.velocity.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN V");
                // }
                // if filter.orientation.coords.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN O");
                // }
                // if filter.accel_bias.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN A");
                // }
                // if filter.rot_bias.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN R");
                // }
                // if filter.gravity.iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN G");
                // }
                // if filter.position_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN PU");
                // }
                // if filter.velocity_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN VU");
                // }
                // if filter.orientation_uncertainty().iter().any(|x| x.is_nan()) {
                //     println!("{:?}", before);
                //     panic!("NaN OU");
                // }
                // filter.observe_position(
                //     Point3::new(longitude as f32, latitude as f32, altitude as f32),
                //     gps_variance,
                // )
                //     .expect("Failed to observe GPS position");
                // println!("{} {} {}", x-x_offset, y-y_offset, z-z_offset);
            }
            _ => {}
        }
    }

    return vec![];

    // let mut current_state = State {
    //     position: Vec3D::ZERO,
    //     velocity: Vec3D::ZERO,
    //     acceleration: Vec3D::ZERO,
    //     orientation: Quaternion::IDENTITY,
    // };
    //
    // let mut result = vec![];
    //
    // loop {
    //     break;
    // }
    //
    // result.push(current_state);
    //
    // return result;
}

/*

State:
position
velocity
orientation
gravity

position_{n} = position_{n-1}

 */
