mod sim;

use crate::sim::{drone::Drone, rigidbody::RigidBody, world::World};
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
                drone.set_thrust(0, 4.905);
                drone.set_thrust(1, 4.905);
                drone.set_thrust(2, 4.905);
                drone.set_thrust(3, 4.905);
            } else {
                // Increase thrust on one side, decrease on the other.
                // Same total thrust, but now we create torque.
                drone.set_thrust(0, 6.0);
                drone.set_thrust(1, 3.81);
                drone.set_thrust(2, 6.0);
                drone.set_thrust(3, 3.81);
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
