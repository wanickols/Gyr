mod sim;

use crate::sim::{drone::Drone, world::World};
use nalgebra::Vector3;

fn main() {
    let mut world = World::new(9.81);

    world.add_drone(Drone::new(
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        2.0,
    ));

    let dt = 0.01;
    let duration = 10.0;
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
            }
        }
        world.step(dt);

        if step % 100 == 0 {
            let drone = &mut world.drones[0];

            println!(
                "t={:.0}s | y={:.2} | vy={:.2} | thrust={:.2}",
                time, drone.position.y, drone.velocity.y, drone.thrust.y,
            );
        }

        time += dt;
        step += 1;
    }
}
