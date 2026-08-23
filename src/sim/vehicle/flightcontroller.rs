use nalgebra::{UnitQuaternion, Vector3};

use crate::sim::world::RigidBody;

pub struct FlightCommand {
    pub collective_thrust: f32,
    pub desired_torque: Vector3<f32>,
}

pub struct FlightController {
    target_orientation: UnitQuaternion<f32>,

    kp: f32,
    kd: f32,

    hover_thrust: f32,
}

impl FlightController {
    pub fn new(hover_thrust: f32) -> Self {
        Self {
            target_orientation: UnitQuaternion::identity(),

            kp: 1.0,
            kd: 0.5,

            hover_thrust,
        }
    }

    pub fn set_target_orientation(&mut self, target: UnitQuaternion<f32>) {
        self.target_orientation = target;
    }

    pub fn update(&self, body: &RigidBody) -> FlightCommand {
        let (current_x, _current_y, current_z) = body.orientation.euler_angles();

        let (target_x, _target_y, target_z) = self.target_orientation.euler_angles();

        let error_x = target_x - current_x;
        let error_z = target_z - current_z;

        let torque_x = self.kp * error_x - self.kd * body.angular_velocity.x;

        let torque_z = self.kp * error_z - self.kd * body.angular_velocity.z;

        FlightCommand {
            collective_thrust: self.hover_thrust,
            desired_torque: Vector3::new(torque_x, 0.0, torque_z),
        }
    }
}
