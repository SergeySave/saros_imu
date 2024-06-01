use nalgebra::{Matrix3, Matrix4, UnitQuaternion, Vector3};

// pub fn quaternion_derivative(q: UnitQuaternion<f64>, v: Vector3<f64>) -> SMatrix<f64, 3, 4> {
//     // https://math.stackexchange.com/questions/2713061/jacobian-of-a-quaternion-rotation-wrt-the-quaternion
//
//     let r = q.quaternion();
//     let t = Quaternion::new(0.0, v.x, v.y, v.z);
//     let i = Quaternion::new(0.0, 1.0, 0.0, 0.0);
//     let j = Quaternion::new(0.0, 0.0, 1.0, 0.0);
//     let k = Quaternion::new(0.0, 0.0, 0.0, 1.0);
//
//     let mut result = SMatrix::<f64, 3, 4>::zeros();
//
//     result.column_mut(0).copy_from(&((r*t) - (r*t).conjugate()).as_vector().xyz());
//     result.column_mut(1).copy_from(&((r*t*i).conjugate() - (r*t*i)).as_vector().xyz());
//     result.column_mut(2).copy_from(&((r*t*j).conjugate() - (r*t*j)).as_vector().xyz());
//     result.column_mut(3).copy_from(&((r*t*k).conjugate() - (r*t*k)).as_vector().xyz());
//
//     // result
//
//     // let a = 2.0 * SMatrix::<f64, 3, 4>::new(
//     //     v.x*q.w + v.z*q.j - v.y*q.k, v.x*q.i + v.y*q.j + v.z*q.k,-v.x*q.j + v.y*q.i + v.z*q.w,-v.x*q.k - v.y*q.w + v.z*q.i,
//     //     v.y*q.w + v.x*q.k - v.z*q.i,-v.y*q.i + v.x*q.j - v.z*q.w,v.x*q.i + v.y*q.j + v.z*q.k,v.x*q.w - v.y*q.k + v.z*q.j,
//     //     v.z*q.w + v.y*q.i - v.x*q.j,-v.z*q.i + v.x*q.k - v.y*q.w,-v.x*q.w + v.y*q.k - v.z*q.j,v.x*q.i + v.y*q.j + v.z*q.k,
//     // );
//     //
//     // println!("a: {}", a);
//     // println!("b: {}", result);
//
//     //         v.x*q.w + v.z*q.j - v.y*q.k, v.x*q.i + v.y*q.j + v.z*q.k,-v.x*q.j + v.y*q.i + v.z*q.w,-v.x*q.k - v.y*q.w + v.z*q.i,
//     //         v.y*q.w + v.x*q.k - v.z*q.i,-v.y*q.i + v.x*q.j - v.z*q.w,v.x*q.i + v.y*q.j + v.z*q.k,v.x*q.w - v.y*q.k + v.z*q.j,
//     //         v.z*q.w + v.y*q.i - v.x*q.j,-v.z*q.i + v.x*q.k - v.y*q.w,-v.x*q.w + v.y*q.k - v.z*q.j,v.x*q.i + v.y*q.j + v.z*q.k,
//     result
// }
//
// pub fn quaternion_product_derivative(r: UnitQuaternion<f64>, q: UnitQuaternion<f64>) -> Matrix4<f64> {
//
//     let r = r.quaternion();
//     let i = Quaternion::new(0.0, 1.0, 0.0, 0.0);
//     let j = Quaternion::new(0.0, 0.0, 1.0, 0.0);
//     let k = Quaternion::new(0.0, 0.0, 0.0, 1.0);
//
//     let mut result = Matrix4::<f64>::zeros();
//
//     result.column_mut(0).copy_from(&vector_from_quat(r));
//     result.column_mut(1).copy_from(&vector_from_quat(&(r*i)));
//     result.column_mut(2).copy_from(&vector_from_quat(&(r*j)));
//     result.column_mut(3).copy_from(&vector_from_quat(&(r*k)));
//     result.transpose_mut();
//
//     result
// }
//
// fn vector_from_quat(quat: &Quaternion<f64>) -> Vector4<f64> {
//     Vector4::new(quat.w, quat.i, quat.j, quat.k)
// }

pub fn omega(vec: &Vector3<f64>) -> Matrix4<f64> {
    Matrix4::new(
        0.0, vec.z, -vec.y, vec.x,
        -vec.z, 0.0, vec.x, vec.y,
        vec.y, -vec.x, 0.0, vec.z,
        -vec.x, -vec.y, -vec.z, 0.0,
    )
}

pub fn quaternion_jacobian_left_multiply(q: &UnitQuaternion<f64>, v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        2.0 * q.j * v.y + 2.0 * q.k * v.z, // partial x w.r.t x
        -4.0 * q.j * v.x + 2.0 * q.i * v.y - 2.0 * q.w * v.z, // partial x w.r.t y
        -4.0 * q.k * v.x + 2.0 * q.w * v.y + 2.0 * q.i * v.z, // partial x w.r.t z

        2.0 * q.j * v.x - 4.0 * q.i * v.y + 2.0 * q.w * v.z, // partial y w.r.t x
        2.0 * q.i * v.x + 2.0 * q.k * v.z, // partial y w.r.t y
        -2.0 * q.w * v.x - 4.0 * q.k * v.y + 2.0 * q.j * v.z, // partial y w.r.t z

        2.0 * q.k * v.x - 2.0 * q.w * v.y - 4.0 * q.i * v.z, // partial z w.r.t x
        2.0 * q.w * v.x + 2.0 * q.k * v.y - 4.0 * q.j * v.z, // partial z w.r.t y
        2.0 * q.i * v.x + 2.0 * q.j * v.y, // partial z w.r.t z
    )
}
