use nalgebra::{Unit, UnitQuaternion, Vector3};

//Linear
pub fn apply_force(velocity: &mut Vector3<f32>, force: Vector3<f32>, mass: f32, dt: f32) {
    *velocity += force / mass * dt;
}

pub fn apply_velocity(position: &mut Vector3<f32>, velocity: Vector3<f32>, dt: f32) {
    *position += velocity * dt;
}

pub fn apply_angular_velocity(
    orientation: &mut UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    dt: f32,
) {
    let speed = angular_velocity.norm();

    if speed > 0.0 {
        let axis = Unit::new_normalize(angular_velocity);
        let angle = speed * dt;

        let delta = UnitQuaternion::from_axis_angle(&axis, angle);

        *orientation = delta * *orientation;
    }
}

pub fn apply_torque(
    angular_velocity: &mut Vector3<f32>,
    torque: Vector3<f32>,
    inertia: Vector3<f32>,
    dt: f32,
) {
    *angular_velocity += torque.component_div(&inertia) * dt;
}
