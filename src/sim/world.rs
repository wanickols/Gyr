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

    pub fn step(&mut self, delta_time: f32) {
        for drone in &mut self.drones {
            // Apply gravity to the drone's velocity
            physics::apply_force(
                &mut drone.velocity,
                Vector3::new(0.0, -self.gravity * drone.mass, 0.0),
                drone.mass,
                delta_time,
            );
            drone.update(delta_time);
        }
    }
}
