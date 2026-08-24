use nalgebra::{UnitQuaternion, Vector3};

use crate::sim::world::RigidBody;
use crate::sim::world::world::GRAVITY;

pub struct FlightCommand {
    pub collective_thrust: f32,
    pub desired_torque: Vector3<f32>,
}

pub struct FlightController {
    target_altitude: f32,
    target_orientation: UnitQuaternion<f32>,

    attitude_kp: f32,
    attitude_kd: f32,

    altitude_kp: f32,
    altitude_kd: f32,

    max_thrust: f32,
}

impl FlightController {
    pub fn new(max_thrust: f32) -> Self {
        Self {
            target_altitude: 0.0,
            target_orientation: UnitQuaternion::identity(),

            attitude_kp: 1.0,
            attitude_kd: 0.75,

            altitude_kp: 2.0,
            altitude_kd: 2.0,

            max_thrust,
        }
    }

    pub fn set_target_orientation(&mut self, target: UnitQuaternion<f32>) {
        self.target_orientation = target;
    }

    pub fn set_target_altitude(&mut self, target: f32) {
        self.target_altitude = target;
    }

    pub fn update(&self, body: &RigidBody) -> FlightCommand {
        let desired_torque = self.attitude_control(body);
        let collective_thrust = self.altitude_control(body);

        FlightCommand {
            collective_thrust,
            desired_torque,
        }
    }

    fn altitude_control(&self, body: &RigidBody) -> f32 {
        let altitude_error = self.target_altitude - body.position.y;

        let altitude_correction =
            self.altitude_kp * altitude_error - self.altitude_kd * body.velocity.y;

        let gravity_compensation = body.mass * GRAVITY;
        let collective_thrust =
            (gravity_compensation + altitude_correction).clamp(0.0, self.max_thrust);

        collective_thrust
    }

    fn attitude_control(&self, body: &RigidBody) -> Vector3<f32> {
        let (current_x, _current_y, current_z) = body.orientation.euler_angles();

        let (target_x, _target_y, target_z) = self.target_orientation.euler_angles();

        let error_x = target_x - current_x;
        let error_z = target_z - current_z;

        let torque_x = self.attitude_kp * error_x - self.attitude_kd * body.angular_velocity.x;

        let torque_z = self.attitude_kp * error_z - self.attitude_kd * body.angular_velocity.z;

        Vector3::new(torque_x, 0.0, torque_z)
    }
}
