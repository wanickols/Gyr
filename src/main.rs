mod sim;

use crate::sim::{drone::Drone, world::World};
use nalgebra::{UnitQuaternion, Vector3};

fn main() {
    //Init World
    let mut world = World::new(9.81);

    //Add Drone
    world.add_drone(Drone::new(
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        UnitQuaternion::identity(),
        2.0,
    ));

    test(&mut world, 10.0, 0.01);
}

// Simulate
fn test(world: &mut World, duration: f32, dt: f32) {
    let mut time = 0.0;
    let mut step = 0;

    while time < duration {
        {
            let drone = &mut world.drones[0];

            if time < 3.0 {
                drone.apply_acceleration(Vector3::zeros());
            } else if time < 6.0 {
                drone.apply_acceleration(Vector3::new(0.0, 9.81, 0.0));
            } else {
                drone.apply_acceleration(Vector3::new(0.0, 15.0, 0.0));

                drone.orientation =
                    UnitQuaternion::from_euler_angles(0.0, 0.0, 30_f32.to_radians());
            }
        }

        world.step(dt);

        if step % 100 == 0 {
            let drone = &world.drones[0];
            let thrust_world = drone.orientation * drone.thrust;

            println!(
                "t={:.0}s | pos=({:.2}, {:.2}, {:.2}) | vel=({:.2}, {:.2}, {:.2}) | thrust=({:.2}, {:.2}, {:.2})",
                time,
                drone.position.x,
                drone.position.y,
                drone.position.z,
                drone.velocity.x,
                drone.velocity.y,
                drone.velocity.z,
                thrust_world.x,
                thrust_world.y,
                thrust_world.z,
            );
        }

        time += dt;
        step += 1;
    }
}