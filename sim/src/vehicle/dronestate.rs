use nalgebra::{UnitQuaternion, Vector3};

pub struct DroneState {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}
