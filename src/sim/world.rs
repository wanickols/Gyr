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
            World::apply_gravity(self.gravity, dt, drone);
            World::apply_linear(dt, drone);
            World::apply_rotational(dt, drone);
        }
    }

    fn apply_gravity(gravity: f32, dt: f32, drone: &mut Drone) {
        let gravity_force = Vector3::new(0.0, -gravity * drone.mass, 0.0);
        physics::apply_force(&mut drone.velocity, gravity_force, drone.mass, dt);
    }

    fn apply_linear(dt: f32, drone: &mut Drone) {
        let local_thrust = drone.total_thrust();
        let world_thrust = drone.orientation * local_thrust;

        physics::apply_force(&mut drone.velocity, world_thrust, drone.mass, dt);
        physics::apply_velocity(&mut drone.position, drone.velocity, dt);
    }

    fn apply_rotational(dt: f32, drone: &mut Drone) {
        let total_torque = drone.total_torque();

        physics::apply_torque(&mut drone.angular_velocity, total_torque, drone.inertia, dt);
        physics::apply_angular_velocity(&mut drone.orientation, drone.angular_velocity, dt);
    }
}
