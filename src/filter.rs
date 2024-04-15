use std::time::Duration;

use nalgebra::{SMatrix, Vector3};

use crate::state;
use crate::state::State;

const STEP_LENGTH: Duration = Duration::from_millis(5);

pub struct Filter {
    state: State,
    covariance: SMatrix<f64, { state::N }, { state::N }>,
    process_noise: SMatrix<f64, { state::N }, { state::N }>,
    // imu_noise_covariance: SMatrix<f64, { state::M_IMU }, { state::M_IMU }>,
    gps_noise_covariance: SMatrix<f64, { state::M_GPS }, { state::M_GPS }>,
}

impl Filter {

    pub fn new() -> Self {
        let mut covariance = SMatrix::<f64, { state::N }, { state::N }>::zeros();
        covariance.fixed_view_mut::<3, 3>(0,0).fill_diagonal(1.0);
        covariance.fixed_view_mut::<3, 3>(3,3).fill_diagonal(1.0);
        covariance.fixed_view_mut::<4, 4>(6,6).fill_diagonal(1.0);
        covariance.fixed_view_mut::<3, 3>(10,10).fill_diagonal(1.0);
        Self {
            state: State::new(),
            covariance,
            process_noise: SMatrix::from_diagonal_element(0.01),
            gps_noise_covariance: SMatrix::from_diagonal_element(10.0),
        }
    }

    pub fn get_state(&self) -> State {
        self.state.clone()
    }

    pub fn step_imu(&mut self, accel: Vector3<f64>, gyro: Vector3<f64>) {
        // Predict Step
        let state_transition_jacobian = self.state.state_transition_jacobian(accel, gyro, STEP_LENGTH);
        self.state = self.state.propagate(accel, gyro, STEP_LENGTH);
        self.covariance = state_transition_jacobian * self.covariance * state_transition_jacobian.transpose() + self.process_noise;
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
}

