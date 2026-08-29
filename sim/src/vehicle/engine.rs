use nalgebra::Vector3;

pub struct Engine {
    pub position: Vector3<f32>,
    pub thrust: f32,
}

impl Engine {
    pub fn new(position: Vector3<f32>) -> Self {
        Engine {
            position,
            thrust: 0.0,
        }
    }

    pub fn torque(&self) -> Vector3<f32> {
        self.position.cross(&self.force())
    }

    pub fn set_thrust(&mut self, thrust: f32) {
        self.thrust = thrust;
    }

    //Private
    pub fn force(&self) -> Vector3<f32> {
        Vector3::new(0.0, self.thrust, 0.0)
    }
}
