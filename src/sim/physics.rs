use nalgebra::Vector3;

pub fn apply_force(velocity: &mut Vector3<f32>, force: Vector3<f32>, mass: f32, dt: f32) {
    let acceleration = force / mass;
    *velocity += acceleration * dt;
}
