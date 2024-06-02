use std::os::macos::raw::stat;
use map_3d::{ecef2geodetic, uvw2enu};
use nalgebra::Vector2;
use serde::Serialize;

// use saros_imu::ELLIPSOID;
use saros_imu::state::State;

#[derive(Serialize)]
pub struct CsvRow {
    // position_x: f64,
    // position_y: f64,
    // position_z: f64,
    //
    // latitude_rad: f64,
    // latitude_deg: f64,
    // longitude_rad: f64,
    // longitude_deg: f64,
    // altitude: f64,
    //
    // velocity_x: f64,
    // velocity_y: f64,
    // velocity_z: f64,
    //
    // velocity_east: f64,
    // velocity_north: f64,
    // velocity_up: f64,
    //
    // course: f64,
    // ground_speed: f64,
    // speed: f64,

    orientation_w: f64,
    orientation_i: f64,
    orientation_j: f64,
    orientation_k: f64,

    roll: f64,
    pitch: f64,
    yaw: f64,

    gravity_x: f64,
    gravity_y: f64,
    gravity_z: f64,

    // gravity_angle: f64,

    magneto_x: f64,
    magneto_y: f64,
    magneto_z: f64,

    // magneto_angle: f64,
}

impl From<State> for CsvRow {
    fn from(state: State) -> Self {
        // let (lat, lon, alt) = ecef2geodetic(state.position.x, state.position.y, state.position.z, ELLIPSOID);
        // let (east, north, up) = uvw2enu(state.velocity.x, state.velocity.y, state.velocity.z, lat, lon);
        // let mut course = east.atan2(north).to_degrees();
        // if course < 0.0 {
        //     course += 360.0;
        // }

        let (roll, pitch, yaw) = state.orientation.euler_angles();

        Self {
            // position_x: state.position.x,
            // position_y: state.position.y,
            // position_z: state.position.z,
            //
            // latitude_rad: lat,
            // latitude_deg: lat.to_degrees(),
            // longitude_rad: lon,
            // longitude_deg: lon.to_degrees(),
            // altitude: alt,
            //
            // velocity_x: state.velocity.x,
            // velocity_y: state.velocity.y,
            // velocity_z: state.velocity.z,
            //
            // velocity_east: east,
            // velocity_north: north,
            // velocity_up: up,
            //
            // course,
            // ground_speed: Vector2::new(east, north).magnitude(),
            // speed: state.velocity.magnitude(),

            orientation_w: state.orientation.w,
            orientation_i: state.orientation.i,
            orientation_j: state.orientation.j,
            orientation_k: state.orientation.k,

            roll,
            pitch,
            yaw,

            gravity_x: state.gravity.x,
            gravity_y: state.gravity.y,
            gravity_z: state.gravity.z,

            // gravity_magnitude: state.gravity.magnitude(),
            // gravity_angle: state.gravity_angle,

            magneto_x: state.magneto.x,
            magneto_y: state.magneto.y,
            magneto_z: state.magneto.z,

            // magneto_magnitude: state.magneto.magnitude(),
            // magneto_angle: state.magneto_angle,
        }
    }
}
