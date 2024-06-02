use std::time::Duration;

use igrf::declination;
use map_3d::ecef2geodetic;
use nalgebra::{Matrix3, Normed, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3, Vector4};
use serde::de::Unexpected::Unit;
use time::Date;

// use crate::ELLIPSOID;
// use crate::jacobian::{omega, quaternion_jacobian_left_multiply};

// pub const COV_N: usize = 3 + 3 + 3 + 3 + 3;
// pub const COV_POSITION: usize = 0;
// pub const COV_VELOCITY: usize = 3;
// pub const COV_ORIENTATION: usize = 6;
// pub const COV_GYRO_BIAS: usize = 9;
// pub const COV_GRAVITY: usize = 12;
// // pub const M_IMU: usize = 3 + 3;
// pub const M_GPS: usize = 3;
// pub const M_MAGNETOMETER: usize = 3;

#[derive(Clone)]
pub struct State {
    pub orientation: UnitQuaternion<f64>,
    pub gravity: Vector3<f64>,
    pub magneto: Vector3<f64>,
    pub gravity_angle: f64,
    pub magneto_angle: f64,
    // pub position: Vector3<f64>,
    // pub velocity: Vector3<f64>,
    // pub orientation: UnitQuaternion<f64>,
    // pub gyro_bias: Vector3<f64>,
    // pub last_gyro: Vector3<f64>,
    // pub gravity: Vector3<f64>,
}

impl State {

    pub fn new() -> Self {
        Self {
            // position: Default::default(),
            // velocity: Default::default(),
            orientation: UnitQuaternion::identity(),
            // gyro_bias: Vector3::new(0.0, 0.0, 0.0),
            // last_gyro: Vector3::new(0.0, 0.0, 0.0),
            gravity: Vector3::new(0.0, 0.0, 1.0),
            magneto: Vector3::new(0.0, 1.0, 0.0),
            gravity_angle: 0.0,
            magneto_angle: 0.0,
        }
    }

    pub fn update_gyro(&mut self, gyro: Vector3<f64>, duration: Duration) {
        // -gy, -gx, gz
        // -ay, -ax, az
        //  mx,  my, mz
        let gyro = UnitQuaternion::from_euler_angles(-gyro.x * duration.as_secs_f64(), gyro.y * duration.as_secs_f64(), gyro.z * duration.as_secs_f64());
        self.orientation = gyro * self.orientation;
    }

    pub fn update_accel(&mut self, accel: Vector3<f64>, fix: f64, update: f64) {
        let accel = Vector3::new(-accel.x, accel.y, accel.z);
        let world = self.orientation * accel;
        let rotation = UnitQuaternion::rotation_between(&world, &self.gravity).unwrap();
        self.gravity_angle = rotation.angle();
        let fix_rotation = UnitQuaternion::from_scaled_axis(rotation.scaled_axis() * fix);
        let update_rotation = UnitQuaternion::from_scaled_axis(rotation.inverse().scaled_axis() * update);
        self.orientation = fix_rotation * self.orientation;
        self.gravity = update_rotation * self.gravity;
    }

    pub fn update_magneto(&mut self, magento: Vector3<f64>, fix: f64, update: f64) {
        let world = self.orientation * magento;
        let rotation = UnitQuaternion::rotation_between(&world, &self.magneto).unwrap();
        self.magneto_angle = rotation.angle();
        let fix_rotation = UnitQuaternion::from_scaled_axis(rotation.scaled_axis() * fix);
        let update_rotation = UnitQuaternion::from_scaled_axis(rotation.inverse().scaled_axis() * update);
        self.orientation = fix_rotation * self.orientation;
        self.magneto = update_rotation * self.magneto;
    }

    // pub fn propagate(&self, acceleration: Vector3<f64>, gyro: Vector3<f64>, delta_t: Duration) -> Self {
    //     let initial_quaternion = self.orientation;
    //
    //     // Propagate the bias (equation 193)
    //     let new_gyro_bias = self.gyro_bias;
    //
    //     // Estimate turn rate (equation 194)
    //     let estimated_new_turn_rate = gyro - self.gyro_bias;
    //     let estimated_old_turn_rate = self.last_gyro - self.gyro_bias;
    //     let average_turn_rate = (estimated_new_turn_rate + estimated_old_turn_rate) / 2.0;
    //
    //     // Propagate quaternion via first order integrator (section 1.6.2)
    //     // exp( 1/2 omega ( w bar ) delta_t )
    //     let left_term = (omega(&average_turn_rate) * 0.5 * delta_t.as_secs_f64()).exp();
    //     // 1/48 * (omega(w_new)omega(w_old) - omega(w_old)omega(w_new)) * delta_t^2
    //     let omega_new = omega(&estimated_new_turn_rate);
    //     let omega_old = omega(&estimated_old_turn_rate);
    //     let right_term = (1.0 / 48.0) * (omega_new*omega_old - omega_old*omega_new) * delta_t.as_secs_f64().powi(2);
    //
    //     // equation 110
    //     let final_rotation = (left_term + right_term) * Vector4::new(self.orientation.i, self.orientation.j, self.orientation.k, self.orientation.w);
    //     let final_quaternion = UnitQuaternion::from_quaternion(Quaternion::new(final_rotation.w, final_rotation.x, final_rotation.y, final_rotation.z));
    //
    //     Self {
    //         position: self.position + self.velocity * delta_t.as_secs_f64() + (initial_quaternion * acceleration + self.gravity) * delta_t.as_secs_f64().powi(2) / 2.0,
    //         velocity: self.velocity + (initial_quaternion * acceleration + self.gravity) * delta_t.as_secs_f64(),
    //         orientation: final_quaternion,
    //         gyro_bias: new_gyro_bias,
    //         last_gyro: gyro,
    //         gravity: self.gravity,
    //     }
    // }
    //
    // pub fn add_state_vector(&mut self, state_vector: &SVector<f64, COV_N>) {
    //     let delta_position = state_vector.fixed_view::<3, 1>(COV_POSITION, 0);
    //     let delta_velocity = state_vector.fixed_view::<3, 1>(COV_VELOCITY, 0);
    //     let delta_orientation = state_vector.fixed_view::<3, 1>(COV_ORIENTATION, 0);
    //     let delta_gyro_bias = state_vector.fixed_view::<3, 1>(COV_GYRO_BIAS, 0);
    //     let delta_gravity = state_vector.fixed_view::<3, 1>(COV_GRAVITY, 0);
    //
    //     let dq_plus = delta_orientation / 2.0;
    //
    //     let dqtdq = (dq_plus.transpose() * dq_plus).x;
    //
    //     let dq_mul = if dqtdq > 1.0 {
    //         (1.0 / (1.0 + dqtdq).sqrt()) * Vector4::new(dq_plus.x, dq_plus.y, dq_plus.z, 1.0)
    //     } else {
    //         Vector4::new(dq_plus.x, dq_plus.y, dq_plus.z, (1.0 - dqtdq).sqrt())
    //     };
    //
    //     let dq_mul = Quaternion::from_vector(dq_mul);
    //     let dq_mul = UnitQuaternion::new_unchecked(dq_mul);
    //
    //     self.position += delta_position;
    //     self.velocity += delta_velocity;
    //     self.orientation = dq_mul * self.orientation;
    //     self.gyro_bias += delta_gyro_bias;
    //     self.gravity += delta_gravity;
    // }
    //
    // pub fn state_transition_jacobian(&self, acceleration: Vector3<f64>, gyro: Vector3<f64>, delta_t: Duration) -> SMatrix<f64, COV_N, COV_N> {
    //     let estimated_new_turn_rate = gyro - self.gyro_bias;
    //     // let turn_rate_magnitude = estimated_new_turn_rate.magnitude();
    //     // let turn_rate_magnitude_time = turn_rate_magnitude * delta_t.as_secs_f64();
    //     // let normalized_turn_rate = if turn_rate_magnitude != 0.0 { estimated_new_turn_rate / turn_rate_magnitude } else { Vector3::zeros() };
    //     let orientation_acceleration_jacobian = quaternion_jacobian_left_multiply(&self.orientation, &acceleration);
    //     // let orientation_acceleration_jacobian = Matrix3::new(
    //     //     2.0 * self.orientation.j * acceleration.y + 2.0 * self.orientation.k * acceleration.z, // partial x w.r.t x
    //     //     -4.0 * self.orientation.j * acceleration.x + 2.0 * self.orientation.i * acceleration.y - 2.0 * self.orientation.w * acceleration.z, // partial x w.r.t y
    //     //     -4.0 * self.orientation.k * acceleration.x + 2.0 * self.orientation.w * acceleration.y + 2.0 * self.orientation.i * acceleration.z, // partial x w.r.t z
    //     //
    //     //     2.0 * self.orientation.j * acceleration.x - 4.0 * self.orientation.i * acceleration.y + 2.0 * self.orientation.w * acceleration.z, // partial y w.r.t x
    //     //     2.0 * self.orientation.i * acceleration.x + 2.0 * self.orientation.k * acceleration.z, // partial y w.r.t y
    //     //     -2.0 * self.orientation.w * acceleration.x - 4.0 * self.orientation.k * acceleration.y + 2.0 * self.orientation.j * acceleration.z, // partial y w.r.t z
    //     //
    //     //     2.0 * self.orientation.k * acceleration.x - 2.0 * self.orientation.w * acceleration.y - 4.0 * self.orientation.i * acceleration.z, // partial z w.r.t x
    //     //     2.0 * self.orientation.w * acceleration.x + 2.0 * self.orientation.k * acceleration.y - 4.0 * self.orientation.j * acceleration.z, // partial z w.r.t y
    //     //     2.0 * self.orientation.i * acceleration.x + 2.0 * self.orientation.j * acceleration.y, // partial z w.r.t z
    //     // );
    //
    //     let identity3 = Matrix3::from_diagonal_element(1.0);
    //     // let normalized_turn_rate_matrix = Matrix3::new(
    //     //     0.0, -normalized_turn_rate.z, normalized_turn_rate.y,
    //     //     normalized_turn_rate.z, 0.0, -normalized_turn_rate.x,
    //     //     -normalized_turn_rate.y, normalized_turn_rate.x, 0.0,
    //     // );
    //     let estimated_new_turn_rate_matrix = Matrix3::new(
    //         0.0, -estimated_new_turn_rate.z, estimated_new_turn_rate.y,
    //         estimated_new_turn_rate.z, 0.0, -estimated_new_turn_rate.x,
    //         -estimated_new_turn_rate.y, estimated_new_turn_rate.x, 0.0,
    //     );
    //
    //     // let theta = turn_rate_magnitude_time.cos() * identity3
    //     //     - turn_rate_magnitude_time.sin()*normalized_turn_rate_matrix
    //     //     + (1.0 - turn_rate_magnitude.cos())*normalized_turn_rate*normalized_turn_rate.transpose();
    //     // let thetav2 = Rotation3::from_scaled_axis(estimated_new_turn_rate * delta_t.as_secs_f64());
    //
    //     // Using the limit expression: We are assuming turn_rate_magnitude is small - see the paper for info on this approximation
    //     let theta = identity3 - estimated_new_turn_rate_matrix*delta_t.as_secs_f64() + estimated_new_turn_rate_matrix.pow(2)*delta_t.as_secs_f64().powi(2)/2.0;
    //     // let psi = -identity3*delta_t.as_secs_f64()
    //     //     + (1.0 / turn_rate_magnitude.powi(2))*(1.0 - turn_rate_magnitude.cos())*estimated_new_turn_rate_matrix
    //     //     - (1.0 / turn_rate_magnitude.powi(3))*(turn_rate_magnitude_time - turn_rate_magnitude.sin())*estimated_new_turn_rate_matrix.pow(2);
    //     let psi = -identity3*delta_t.as_secs_f64() + estimated_new_turn_rate_matrix*delta_t.as_secs_f64().powi(2)/2.0 - estimated_new_turn_rate_matrix.pow(2)*delta_t.as_secs_f64().powi(3)/6.0;
    //
    //     let mut jacobian = SMatrix::<f64, COV_N, COV_N>::zeros();
    //
    //     jacobian.fixed_view_mut::<3, 3>(COV_POSITION, COV_POSITION).copy_from(&identity3);
    //     jacobian.fixed_view_mut::<3, 3>(COV_POSITION, COV_VELOCITY).copy_from(&Matrix3::from_diagonal_element(delta_t.as_secs_f64()));
    //     jacobian.fixed_view_mut::<3, 3>(COV_POSITION, COV_ORIENTATION).copy_from(&(orientation_acceleration_jacobian * delta_t.as_secs_f64().powi(2) / 2.0));
    //     jacobian.fixed_view_mut::<3, 3>(COV_POSITION, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_POSITION, COV_GRAVITY).copy_from(&Matrix3::from_diagonal_element(delta_t.as_secs_f64().powi(2) / 2.0));
    //
    //     jacobian.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_POSITION).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_VELOCITY).copy_from(&identity3);
    //     jacobian.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_ORIENTATION).copy_from(&(orientation_acceleration_jacobian * delta_t.as_secs_f64()));
    //     jacobian.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_GRAVITY).copy_from(&Matrix3::from_diagonal_element(delta_t.as_secs_f64()));
    //
    //     jacobian.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_POSITION).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_ORIENTATION).copy_from(&theta);
    //     jacobian.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_GYRO_BIAS).copy_from(&psi);
    //     jacobian.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     jacobian.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_POSITION).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_ORIENTATION).copy_from(&Matrix3::zeros()); // 0
    //     jacobian.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_GYRO_BIAS).copy_from(&identity3);
    //     jacobian.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     jacobian.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_POSITION).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_ORIENTATION).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     jacobian.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_GRAVITY).copy_from(&identity3);
    //
    //     jacobian
    // }
    //
    // pub fn process_noise(&self, _: Vector3<f64>, gyro: Vector3<f64>, delta_t: Duration) -> SMatrix<f64, COV_N, COV_N> {
    //     const POSITION_NOISE: f64 = 0.001;
    //     const VELOCITY_NOISE: f64 = 0.001;
    //     const ORIENTATION_NOISE: f64 = 0.001;
    //     const GYRO_BIAS_NOISE: f64 = 0.001;
    //     const GRAVITY_NOISE: f64 = 0.001;
    //
    //     let estimated_new_turn_rate = gyro - self.gyro_bias;
    //     let turn_rate_magnitude = estimated_new_turn_rate.magnitude();
    //     let turn_rate_magnitude_time = turn_rate_magnitude * delta_t.as_secs_f64();
    //
    //     let identity3 = Matrix3::from_diagonal_element(1.0);
    //     let estimated_new_turn_rate_matrix = Matrix3::new(
    //         0.0, -estimated_new_turn_rate.z, estimated_new_turn_rate.y,
    //         estimated_new_turn_rate.z, 0.0, -estimated_new_turn_rate.x,
    //         -estimated_new_turn_rate.y, estimated_new_turn_rate.x, 0.0,
    //     );
    //
    //     let q11 = ORIENTATION_NOISE.powi(2)*delta_t.as_secs_f64()* identity3 + GYRO_BIAS_NOISE.powi(2)*(identity3*(delta_t.as_secs_f64().powi(3)/3.0) + ((turn_rate_magnitude_time.powi(3)/3.0 + 2.0*turn_rate_magnitude_time.sin() - 2.0*turn_rate_magnitude_time)/(turn_rate_magnitude.powi(5)))*estimated_new_turn_rate_matrix.pow(2));
    //     let q12 = -GYRO_BIAS_NOISE.powi(2)*(
    //         identity3*(delta_t.as_secs_f64().powi(2)/2.0)
    //         - ((turn_rate_magnitude_time - turn_rate_magnitude_time.sin())/(turn_rate_magnitude.powi(3)))*estimated_new_turn_rate_matrix
    //         + ((turn_rate_magnitude_time.powi(2)/2.0 + turn_rate_magnitude_time.cos() - 1.0)/(turn_rate_magnitude.powi(4)))*estimated_new_turn_rate_matrix.pow(2)
    //     );
    //     let q22 = GYRO_BIAS_NOISE.powi(2)*delta_t.as_secs_f64() * identity3;
    //
    //     let mut noise = SMatrix::<f64, COV_N, COV_N>::zeros();
    //
    //     noise.fixed_view_mut::<3, 3>(COV_POSITION, COV_POSITION).copy_from(&Matrix3::from_diagonal_element(POSITION_NOISE.powi(2)));
    //     noise.fixed_view_mut::<3, 3>(COV_POSITION, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_POSITION, COV_ORIENTATION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_POSITION, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_POSITION, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     noise.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_POSITION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_VELOCITY).copy_from(&Matrix3::from_diagonal_element(VELOCITY_NOISE.powi(2)));
    //     noise.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_ORIENTATION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_VELOCITY, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     noise.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_POSITION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_ORIENTATION).copy_from(&q11);
    //     noise.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_GYRO_BIAS).copy_from(&q12);
    //     noise.fixed_view_mut::<3, 3>(COV_ORIENTATION, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     noise.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_POSITION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_ORIENTATION).copy_from(&q12.transpose());
    //     noise.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_GYRO_BIAS).copy_from(&q22);
    //     noise.fixed_view_mut::<3, 3>(COV_GYRO_BIAS, COV_GRAVITY).copy_from(&Matrix3::zeros());
    //
    //     noise.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_POSITION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_VELOCITY).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_ORIENTATION).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_GYRO_BIAS).copy_from(&Matrix3::zeros());
    //     noise.fixed_view_mut::<3, 3>(COV_GRAVITY, COV_GRAVITY).copy_from(&Matrix3::from_diagonal_element(GRAVITY_NOISE.powi(2)));
    //
    //     noise
    // }
    //
    // pub fn predict_gps_measurement(&self) -> SVector<f64, M_GPS> {
    //     let mut predicted = SVector::<f64, M_GPS>::zeros();
    //
    //     predicted.fixed_view_mut::<3, 1>(0, 0).copy_from(&self.position);
    //
    //     predicted
    // }
    //
    // pub fn gps_measurement_jacobian(&self) -> SMatrix<f64, M_GPS, COV_N> {
    //     let mut jacobian = SMatrix::<f64, M_GPS, COV_N>::zeros();
    //
    //     // GPS position w.r.t. position
    //     jacobian.fixed_view_mut::<3, 3>(0, COV_POSITION).fill_with_identity();
    //
    //     jacobian
    // }
    //
    // fn expected_magnetic_field_ecef(&self, date: Date) -> SVector<f64, M_MAGNETOMETER> {
    //     let (lat, lon, alt) = ecef2geodetic(self.position.x, self.position.y, self.position.z, ELLIPSOID);
    //
    //     // This is in the NED reference frame
    //     let field = declination(lat, lon, alt as u32, date).expect("Failed to get magnetic field");
    //     let field_direction = Vector3::new(field.x, field.y, field.z).normalize();
    //
    //     // Compute the NED reference frame axes at our position
    //     let down = -self.position.normalize();
    //     let east = down.cross(&Vector3::new(0.0, 0.0, 1.0)).normalize();
    //     let north = east.cross(&down).normalize();
    //
    //     let field = field_direction.x*north + field_direction.y*east + field_direction.z*down;
    //     field.normalize()
    // }
    //
    // pub fn predict_magnetometer_measurement(&self, date: Date) -> SVector<f64, M_MAGNETOMETER> {
    //     // Orientation * Local Reference Frame = Global Reference Frame (ECEF)
    //     // Orientation * Magnetometer = Magnetic Field
    //     // Orientation^-1 * Orientation * Magnetometer = Orientation^-1 * Magnetic Field
    //     // Magnetometer = Orientation^-1 * Magnetic Field
    //     let field = self.expected_magnetic_field_ecef(date);
    //     // Convert to local reference frame
    //     let prediction = self.orientation.inverse() * field;
    //
    //     prediction
    // }
    //
    // pub fn magnetometer_measurement_jacobian(&self, date: Date) -> SMatrix<f64, M_MAGNETOMETER, COV_N> {
    //     // For simplicity assume partial magnetometer w.r.t. position = 0
    //     // This should be true over a small enough patch of the earth
    //
    //     let field = self.expected_magnetic_field_ecef(date);
    //     // Negated due to the inverse
    //     let orientation_magnetometer_jacobian = -quaternion_jacobian_left_multiply(&self.orientation.inverse(), &field);
    //
    //     let mut jacobian = SMatrix::<f64, M_MAGNETOMETER, COV_N>::zeros();
    //
    //     // Magnetometer w.r.t orientation
    //     jacobian.fixed_view_mut::<3, 3>(0, COV_ORIENTATION).copy_from(&orientation_magnetometer_jacobian);
    //
    //     jacobian
    // }
}
