use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct Drone {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub mass: f32,
    pub thrust: Vector3<f32>,
}

impl Drone {
    pub fn new(position: Vector3<f32>, velocity: Vector3<f32>, mass: f32 ) -> Self {
        Drone {
            position,
            velocity,
            mass,
            thrust: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        self.velocity += self.thrust / self.mass * delta_time;
        self.position += self.velocity * delta_time;
    }

    pub fn apply_acceleration(&mut self, acceleration: Vector3<f32>) {
        self.thrust = acceleration * self.mass;
    }

    #[allow(dead_code)]
    pub fn apply_thrust(&mut self, thrust: Vector3<f32>) {
        self.thrust = thrust;
    }
}
