use crate::sim::vehicle::Drone;
use crate::sim::world::{RigidBody, World};
use nalgebra::{UnitQuaternion, Vector3};
pub struct Simulation {
    pub world: World,
}

impl Simulation {
    pub fn new() -> Self {
        Simulation {
            world: World::new(),
        }
    }

    pub fn sim_drone(&mut self, duration: f32, dt: f32) {
        let body_id = self.world.add_body(RigidBody::new(
            Vector3::new(0.0, 10.0, 0.0),
            UnitQuaternion::from_euler_angles(4.0_f32.to_radians(), 0.0, 0.0),
            2.0,
        ));

        let mut falcon = Drone::new(body_id, 25.0);

        self.test(duration, dt, &mut falcon);
    }
    // Simulate
    fn test(&mut self, duration: f32, dt: f32, drone: &mut Drone) {
        let mut time = 0.0;
        let mut step = 0;
        drone
            .flightcontroller
            .set_target_orientation(UnitQuaternion::identity());

        drone.flightcontroller.set_target_altitude(15.0);
        while time < duration {
            drone.update(&mut self.world);
            if step % 100 == 0 {
                let body = self.world.mut_body(&drone.body);
                body.print_state(time);
            }

            self.world.step(dt);

            time += dt;
            step += 1;
        }
    }
}
