mod sim;

use crate::sim::vehicle::Drone;
use crate::sim::world::{RigidBody, World};
use nalgebra::{UnitQuaternion, Vector3};

fn main() {
    //Init World
    let mut world = World::new(9.81);

    let body_id = world.add_body(RigidBody::new(
        Vector3::new(0.0, 10.0, 0.0),
        UnitQuaternion::identity(),
        2.0,
    ));

    let mut falcon = Drone::new(body_id);

    test(&mut world, 10.0, 0.01, &mut falcon);
}

// Simulate
fn test(world: &mut World, duration: f32, dt: f32, drone: &mut Drone) {
    let mut time = 0.0;
    let mut step = 0;

    while time < duration {
        {
            if time < 3.0 {
                // Hover: 19.62 N total for a 2 kg drone
                drone.set_motor(19.62, Vector3::new(0.0, 0.0, 0.0));
            } else {
                drone.set_motor(19.62, Vector3::new(0.0, 0.0, 2.0));
            }
        }

        drone.update(world);
        if step % 100 == 0 {
            let body = world.mut_body(&drone.body);
            body.print_state(time);
        }

        world.step(dt);

        time += dt;
        step += 1;
    }
}
