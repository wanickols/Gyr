use crate::vehicle::{Drone, DroneState};
use crate::world::{RigidBody, World};
use nalgebra::{UnitQuaternion, Vector3};
pub struct Simulation {
    world: World,
    falcon: Drone,
}

impl Simulation {
    pub fn new() -> Self {
        let mut world = World::new();
        let body_id = world.add_body(RigidBody::new(
            Vector3::new(0.0, 10.0, 0.0),
            UnitQuaternion::from_euler_angles(4.0_f32.to_radians(), 0.0, 0.0),
            2.0,
        ));

        Simulation {
            world: world,
            falcon: Drone::new(body_id, 25.0),
        }
    }

    //Bevy to step through
    pub fn step(&mut self, dt: f32) {
        self.apply_controls();
        self.integrate(dt);
    }

    //Simulator
    pub fn test(&mut self, duration: f32, dt: f32) {
        let mut time = 0.0;
        let mut step = 0;

        self.set_targets(15.0, UnitQuaternion::identity());

        while time < duration {
            self.apply_controls();

            if step % 100 == 0 {
                let body = self.world.mut_body(&self.falcon.body);
                body.print_state(time);
            }

            self.integrate(dt);

            time += dt;
            step += 1;
        }
    }

    pub fn set_altitude_with_default_ori(&mut self, target_altitude: f32) {
        self.set_targets(target_altitude, UnitQuaternion::identity());
    }
    //Internals
    fn set_targets(&mut self, target_altitude: f32, target_orientation: UnitQuaternion<f32>) {
        let drone = &mut self.falcon;

        drone.flightcontroller.set_target_altitude(target_altitude);
        drone
            .flightcontroller
            .set_target_orientation(target_orientation);
    }

    fn apply_controls(&mut self) {
        self.falcon.update(&mut self.world);
    }

    fn integrate(&mut self, dt: f32) {
        self.world.step(dt);
    }

    //Getters
    pub fn world(&self) -> &World {
        &self.world
    }

    pub fn drone_state(&self) -> DroneState {
        let body = self.world.ref_body(&self.falcon.body);

        DroneState {
            position: body.position,
            orientation: body.orientation,
        }
    }
}
