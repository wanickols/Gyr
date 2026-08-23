use crate::sim::rigidbody::BodyId;
use crate::sim::world::World;
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
    fn force(&self) -> Vector3<f32> {
        Vector3::new(0.0, self.thrust, 0.0)
    }
}

pub struct Drone {
    pub body: BodyId,

    // Components
    pub engines: [Engine; 4],
}

impl Drone {
    pub fn new(body: BodyId) -> Self {
        Drone {
            body,
            engines: Drone::init_engines(),
        }
    }

    pub fn update(&mut self, world: &mut World) {
        let body = world.mut_body(&self.body);

        for engine in &self.engines {
            let local_force = engine.force();
            let world_force = body.orientation.transform_vector(&local_force);

            let local_position = engine.position;
            let world_position = body.position + body.orientation.transform_vector(&local_position);

            body.apply_force_at_point(world_force, world_position);

            let local_torque = engine.torque();
            let world_torque = body.orientation.transform_vector(&local_torque);

            body.apply_torque(world_torque);
        }
    }

    pub fn set_thrust(&mut self, engine_index: usize, thrust: f32) {
        if engine_index < self.engines.len() {
            self.engines[engine_index].set_thrust(thrust);
        }
    }

    //Private
    fn init_engines() -> [Engine; 4] {
        [
            Engine::new(Vector3::new(-0.5, 0.0, 0.5)),
            Engine::new(Vector3::new(0.5, 0.0, 0.5)),
            Engine::new(Vector3::new(-0.5, 0.0, -0.5)),
            Engine::new(Vector3::new(0.5, 0.0, -0.5)),
        ]
    }
}
