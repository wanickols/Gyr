use crate::sim::{drone::Drone, physics};
use nalgebra::Vector3;

pub struct World {
    pub drones: Vec<Drone>,
    gravity: f32,
}

impl World {
    pub fn new(gravity: f32) -> Self {
        World {
            drones: Vec::new(),
            gravity,
        }
    }

    pub fn add_drone(&mut self, drone: Drone) {
        self.drones.push(drone);
    }

    pub fn step(&mut self, dt: f32) {
        for drone in &mut self.drones {
            // Apply gravity to the drone's velocity
            let gravity_force = Vector3::new(0.0, -self.gravity * drone.mass, 0.0);
            physics::apply_force(&mut drone.velocity, gravity_force, drone.mass, dt);

            let world_thrust = drone.orientation * drone.thrust;
            //apply thrust
            physics::apply_force(&mut drone.velocity, world_thrust, drone.mass, dt);

            //move drone
            physics::integrate_position(&mut drone.position, drone.velocity, dt);
        }
    }
}
