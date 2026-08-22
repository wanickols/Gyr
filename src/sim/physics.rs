use nalgebra::Vector3;

pub fn apply_force(velocity: &mut Vector3<f32>, force: Vector3<f32>, mass: f32, dt: f32) {
    *velocity += force / mass * dt;
}

pub fn integrate_position(position: &mut Vector3<f32>, velocity: Vector3<f32>, dt: f32) {
    *position += velocity * dt;
}
