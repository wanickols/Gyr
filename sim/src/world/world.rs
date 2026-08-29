use crate::world::{BodyId, RigidBody};
use nalgebra::Vector3;

pub static GRAVITY: f32 = 9.81;

pub struct World {
    pub bodies: Vec<RigidBody>,
}

impl World {
    pub fn new() -> Self {
        World { bodies: Vec::new() }
    }

    pub fn add_body(&mut self, body: RigidBody) -> BodyId {
        self.bodies.push(body);
        BodyId(self.bodies.len() - 1)
    }

    pub fn ref_body(&self, body_id: &BodyId) -> &RigidBody {
        &self.bodies[body_id.0]
    }

    pub fn mut_body(&mut self, body_id: &BodyId) -> &mut RigidBody {
        &mut self.bodies[body_id.0]
    }

    pub fn step(&mut self, dt: f32) {
        for body in &mut self.bodies {
            let gravity_force = Vector3::new(0.0, -GRAVITY * body.mass, 0.0);
            body.apply_force(gravity_force);

            body.integrate(dt);
        }
    }
}
