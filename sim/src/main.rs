mod sim;

use crate::sim::vehicle::Drone;
use crate::sim::world::{RigidBody, World};
use nalgebra::{UnitQuaternion, Vector3};

fn main() {
    //Init World
    let mut world = World::new();

    let body_id = world.add_body(RigidBody::new(
        Vector3::new(0.0, 10.0, 0.0),
        UnitQuaternion::from_euler_angles(4.0_f32.to_radians(), 0.0, 0.0),
        2.0,
    ));

    let mut falcon = Drone::new(body_id, 25.0);

    test(&mut world, 40.0, 0.01, &mut falcon);
}

// Simulate
fn test(world: &mut World, duration: f32, dt: f32, drone: &mut Drone) {
    let mut time = 0.0;
    let mut step = 0;
    drone
        .flightcontroller
        .set_target_orientation(UnitQuaternion::identity());

    drone.flightcontroller.set_target_altitude(15.0);
    while time < duration {
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
