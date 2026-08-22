use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct Drone {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    pub mass: f32,
    pub thrust: Vector3<f32>,
}

impl Drone {
    pub fn new(
        position: Vector3<f32>,
        velocity: Vector3<f32>,
        orientation: UnitQuaternion<f32>,
        mass: f32,
    ) -> Self {
        Drone {
            position,
            velocity,
            orientation,
            mass,
            thrust: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn apply_acceleration(&mut self, acceleration: Vector3<f32>) {
        self.apply_thrust(acceleration * self.mass);
    }

    pub fn apply_thrust(&mut self, thrust: Vector3<f32>) {
        self.thrust = thrust;
    }
}
