use nalgebra::Vector3;

use crate::sim::{
    vehicle::{Engine, FlightController, MotorSystem},
    world::{BodyId, World},
};

pub struct Drone {
    pub body: BodyId,

    // Components
    pub motorsystem: MotorSystem,
    pub flightcontroller: FlightController,
}

impl Drone {
    pub fn new(body: BodyId) -> Self {
        let mut motorsystem = MotorSystem::new();

        motorsystem.add_motor(Engine::new(Vector3::new(0.5, 0.0, 0.5)));
        motorsystem.add_motor(Engine::new(Vector3::new(-0.5, 0.0, 0.5)));
        motorsystem.add_motor(Engine::new(Vector3::new(0.5, 0.0, -0.5)));
        motorsystem.add_motor(Engine::new(Vector3::new(-0.5, 0.0, -0.5)));

        Drone {
            body,
            motorsystem: motorsystem,
            flightcontroller: FlightController::new(19.62),
        }
    }

    pub fn update(&mut self, world: &mut World) {
        let body = world.mut_body(&self.body);
        let command = self.flightcontroller.update(body);

        self.motorsystem
            .set_motor(command.collective_thrust, command.desired_torque);

        self.motorsystem.update();

        for engine in &self.motorsystem.motors {
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

    pub fn set_motor(&mut self, collective_thrust: f32, desired_torque: Vector3<f32>) {
        self.motorsystem
            .set_motor(collective_thrust, desired_torque);
    }
}
