use std::time::Duration;

use nalgebra::{Matrix3, SMatrix, SVector, UnitQuaternion, Vector3, Vector4};

pub const N: usize = 3 + 3 + 4 + 3;
// pub const M_IMU: usize = 3 + 3;
pub const M_GPS: usize = 3;

#[derive(Clone)]
pub struct State {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub gravity: Vector3<f64>,
}

impl State {

    pub fn new() -> Self {
        Self {
            position: Default::default(),
            velocity: Default::default(),
            orientation: Default::default(),
            gravity: Default::default(),
        }
    }

    pub fn state_vector(&self) -> SVector<f64, N> {
        let mut state_vector = SVector::<f64, N>::zeros();

        state_vector.fixed_view_mut::<3, 1>(0, 0).copy_from(&self.position);
        state_vector.fixed_view_mut::<3, 1>(3, 0).copy_from(&self.velocity);
        state_vector.fixed_view_mut::<4, 1>(6, 0).copy_from(&self.orientation.coords);
        state_vector.fixed_view_mut::<3, 1>(10, 0).copy_from(&self.gravity);

        state_vector
    }

    pub fn add_state_vector(&mut self, vector: &SVector<f64, N>) {
        self.position += vector.fixed_view::<3, 1>(0, 0);
        self.velocity += vector.fixed_view::<3, 1>(3, 0);
        self.orientation = UnitQuaternion::new_normalize((self.orientation.coords + vector.fixed_view::<4, 1>(6, 0)).into());
        self.gravity += vector.fixed_view::<3, 1>(10, 0);
    }

    pub fn propagate(&self, acceleration: Vector3<f64>, gyro: Vector3<f64>, delta_t: Duration) -> Self {
        Self {
            position: self.position + self.velocity * delta_t.as_secs_f64() + (acceleration - self.gravity) * delta_t.as_secs_f64().powi(2) / 2.0,
            velocity: self.velocity + (acceleration - self.gravity) * delta_t.as_secs_f64(),
            // acceleration: self.acceleration,
            orientation: UnitQuaternion::from_scaled_axis(gyro * delta_t.as_secs_f64()) * self.orientation,
            // rotation_rate: self.rotation_rate,
            gravity: self.gravity,
        }
    }

    pub fn state_transition_jacobian(&self, acceleration: Vector3<f64>, gyro: Vector3<f64>, delta_t: Duration) -> SMatrix<f64, N, N> {
        let mut jacobian = SMatrix::<f64, N, N>::zeros();

        jacobian.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
        jacobian.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::from_diagonal_element(delta_t.as_secs_f64()));
        jacobian.fixed_view_mut::<3, 3>(0, 10).copy_from(&Matrix3::from_diagonal_element(-delta_t.as_secs_f64().powi(2) / 2.0));

        jacobian.fixed_view_mut::<3, 3>(3, 3).fill_with_identity();
        jacobian.fixed_view_mut::<3, 3>(3, 10).copy_from(&Matrix3::from_diagonal_element(-delta_t.as_secs_f64()));

        // jacobian.fixed_view_mut::<3, 3>(6, 6).fill_with_identity();

        let rotate = UnitQuaternion::from_scaled_axis(gyro * delta_t.as_secs_f64());

        // partial of (rotation_rate * orientation) w.r.t orientation
        jacobian.fixed_view_mut::<4,1>(6, 6).copy_from(&Vector4::new(rotate.w, rotate.i, rotate.j, rotate.k));
        jacobian.fixed_view_mut::<4,1>(6, 7).copy_from(&Vector4::new(-rotate.i, rotate.w, rotate.k, -rotate.j));
        jacobian.fixed_view_mut::<4,1>(6, 8).copy_from(&Vector4::new(-rotate.j, -rotate.k, rotate.w, rotate.i));
        jacobian.fixed_view_mut::<4,1>(6, 9).copy_from(&Vector4::new(-rotate.k, rotate.j, -rotate.i, rotate.w));

        // Gravity
        jacobian.fixed_view_mut::<3, 3>(10, 10).fill_with_identity();

        jacobian
    }

    // pub fn predict_imu_measurement(&self) -> SVector<f64, M_IMU> {
    //     let mut predicted = SVector::<f64, M_IMU>::zeros();
    //
    //     predicted.fixed_view_mut::<3, 1>(0, 0).copy_from(&(self.orientation * (self.acceleration - self.gravity)));
    //     predicted.fixed_view_mut::<3, 1>(3, 0).copy_from(&self.rotation_rate);
    //
    //     predicted
    // }

    // pub fn imu_measurement_jacobian(&self) -> SMatrix<f64, M_IMU, N> {
    //     let mut jacobian = SMatrix::<f64, M_IMU, N>::zeros();
    //
    //     // IMU acceleration w.r.t. acceleration
    //     jacobian.fixed_view_mut::<3, 3>(0, 6).copy_from(self.orientation.to_rotation_matrix().matrix());
    //
    //     // IMU acceleration w.r.t. orientation
    //     let v = self.acceleration - self.gravity;
    //     let q = &self.orientation;
    //     jacobian.fixed_view_mut::<3, 4>(0, 9).copy_from(&(2.0 * SMatrix::<f64, 3, 4>::new(
    //         v.x*q.w + v.z*q.j - v.y*q.k, v.x*q.i + v.y*q.j + v.z*q.k,-v.x*q.j + v.y*q.i + v.z*q.w,-v.x*q.k - v.y*q.w + v.z*q.i,
    //         v.y*q.w + v.x*q.k - v.z*q.i,-v.y*q.i + v.x*q.j - v.z*q.w,v.x*q.i + v.y*q.j + v.z*q.k,v.x*q.w - v.y*q.k + v.z*q.j,
    //         v.z*q.w + v.y*q.i - v.x*q.j,-v.z*q.i + v.x*q.k - v.y*q.w,-v.x*q.w + v.y*q.k - v.z*q.j,v.x*q.i + v.y*q.j + v.z*q.k,
    //     )));
    //
    //     // IMU acceleration w.r.t. gravity
    //     jacobian.fixed_view_mut::<3, 3>(0, 16).copy_from(&(-self.orientation.to_rotation_matrix().matrix()));
    //
    //     // IMU gyro w.r.t rotation rate
    //     jacobian.fixed_view_mut::<3, 3>(3, 13).fill_with_identity();
    //
    //     jacobian
    // }

    pub fn predict_gps_measurement(&self) -> SVector<f64, M_GPS> {
        let mut predicted = SVector::<f64, M_GPS>::zeros();

        predicted.fixed_view_mut::<3, 1>(0, 0).copy_from(&self.position);

        predicted
    }

    pub fn gps_measurement_jacobian(&self) -> SMatrix<f64, M_GPS, N> {
        let mut jacobian = SMatrix::<f64, M_GPS, N>::zeros();

        // GPS position w.r.t. position
        jacobian.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();

        jacobian
    }
}
