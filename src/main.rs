mod sim;

use crate::sim::{drone::Drone, world::World};
use nalgebra::{UnitQuaternion, Vector3};

fn main() {
    //Init World
    let mut world = World::new(9.81);

    //Add Drone
    world.add_drone(Drone::new(
        Vector3::new(0.0, 10.0, 0.0),
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

        world.step(dt);

        if step % 100 == 0 {
            let drone = &world.drones[0];

            let local_thrust = drone.total_thrust();
            let world_thrust = drone.orientation * local_thrust;
            let total_torque = drone.total_torque();
            let (roll, pitch, yaw) = drone.orientation.euler_angles();

            println!(
                "\
t={:.0}s
  pos:      ({:>8.2}, {:>8.2}, {:>8.2})
  vel:      ({:>8.2}, {:>8.2}, {:>8.2})
  thrust:   ({:>8.2}, {:>8.2}, {:>8.2})
  torque:   ({:>8.2}, {:>8.2}, {:>8.2})
  ang vel:  ({:>8.2}, {:>8.2}, {:>8.2})
  euler:    ({:>8.2}°, {:>8.2}°, {:>8.2}°)
",
                time,
                drone.position.x,
                drone.position.y,
                drone.position.z,
                drone.velocity.x,
                drone.velocity.y,
                drone.velocity.z,
                world_thrust.x,
                world_thrust.y,
                world_thrust.z,
                total_torque.x,
                total_torque.y,
                total_torque.z,
                drone.angular_velocity.x,
                drone.angular_velocity.y,
                drone.angular_velocity.z,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
            );
        }

        time += dt;
        step += 1;
    }
}
