use std::time::Duration;

use nalgebra::{SMatrix, Vector3};
use time::Date;

use crate::state;
use crate::state::State;

pub(crate) const STEP_LENGTH: Duration = Duration::from_millis(5);

pub struct Filter {
    state: State,
    covariance: SMatrix<f64, { state::COV_N }, { state::COV_N }>,
    // imu_noise_covariance: SMatrix<f64, { state::M_IMU }, { state::M_IMU }>,
    gps_noise_covariance: SMatrix<f64, { state::M_GPS }, { state::M_GPS }>,
    magnetometer_noise_covariance: SMatrix<f64, { state::M_MAGNETOMETER }, { state::M_MAGNETOMETER }>,
}

impl Filter {

    pub fn new() -> Self {
        let mut covariance = SMatrix::<f64, { state::COV_N }, { state::COV_N }>::zeros();
        covariance.fixed_view_mut::<3, 3>(0,0).fill_diagonal(10000.0);
        covariance.fixed_view_mut::<3, 3>(3,3).fill_diagonal(0.01);
        covariance.fixed_view_mut::<3, 3>(6,6).fill_diagonal(1.0);
        covariance.fixed_view_mut::<3, 3>(9,9).fill_diagonal(0.01);
        covariance.fixed_view_mut::<3, 3>(12,12).fill_diagonal(10.0);
        Self {
            state: State::new(),
            covariance,
            gps_noise_covariance: SMatrix::from_diagonal_element(10.0),
            magnetometer_noise_covariance: SMatrix::from_diagonal_element(0.001),
        }
    }

    pub fn get_state(&self) -> State {
        self.state.clone()
    }

    pub fn step_imu(&mut self, accel: Vector3<f64>, gyro: Vector3<f64>) {
        // Predict Step
        let state_transition_jacobian = self.state.state_transition_jacobian(accel, gyro, STEP_LENGTH);
        let process_noise = self.state.process_noise(accel, gyro, STEP_LENGTH);
        self.state = self.state.propagate(accel, gyro, STEP_LENGTH);
        self.covariance = state_transition_jacobian * self.covariance * state_transition_jacobian.transpose() + process_noise;
    }

    pub fn observe_gps(&mut self, position: Vector3<f64>) {
        // GPS Update
        let measurement = position;
        let measurement_residual = measurement - self.state.predict_gps_measurement();
        let observation_jacobian = self.state.gps_measurement_jacobian();
        let residual_covariance = observation_jacobian * self.covariance * observation_jacobian.transpose() + self.gps_noise_covariance;
        let kalman_gain = self.covariance * observation_jacobian.transpose() * residual_covariance.try_inverse().expect("Could not invert gps residual covariance");
        self.state.add_state_vector(&(kalman_gain * measurement_residual));
        self.covariance = (SMatrix::identity() - kalman_gain * observation_jacobian) * self.covariance;
    }

    pub fn observe_magnetometer(&mut self, magnetometer: Vector3<f64>, date: Date) {
        // Magnetometer Update
        let measurement = magnetometer;
        let measurement_residual = measurement.normalize() - self.state.predict_magnetometer_measurement(date).normalize();
        let observation_jacobian = self.state.magnetometer_measurement_jacobian(date);
        let residual_covariance = observation_jacobian * self.covariance * observation_jacobian.transpose() + self.magnetometer_noise_covariance;
        let kalman_gain = self.covariance * observation_jacobian.transpose() * residual_covariance.try_inverse().expect("Could not invert magnetometer residual covariance");
        self.state.add_state_vector(&(kalman_gain * measurement_residual));
        self.covariance = (SMatrix::identity() - kalman_gain * observation_jacobian) * self.covariance;
    }

    // pub fn set_position(&mut self, position: Vector3<f64>) {
    //     self.state.position = position;
    //     self.state.gravity = -position.normalize() * 9.81
    // }
}

