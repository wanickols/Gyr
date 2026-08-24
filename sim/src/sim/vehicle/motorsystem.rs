use crate::sim::vehicle::Engine;
use nalgebra::Vector3;

pub struct MotorSystem {
    pub motors: Vec<Engine>,

    collective_thrust: f32,
    desired_torque: Vector3<f32>,
}

impl MotorSystem {
    pub fn new() -> Self {
        MotorSystem {
            motors: Vec::new(),
            collective_thrust: 0.0,
            desired_torque: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn update(&mut self) {
        let x_arm = self.motors[0].position.z.abs();
        let z_arm = self.motors[0].position.x.abs();

        let x_adjust = self.desired_torque.x / (4.0 * x_arm);
        let z_adjust = self.desired_torque.z / (4.0 * z_arm);

        self.mix(x_adjust, z_adjust);
    }

    fn mix(&mut self, torque_x: f32, torque_z: f32) {
        let base = self.collective_thrust / 4.0;

        self.motors[0].set_thrust(base - torque_x + torque_z);
        self.motors[1].set_thrust(base - torque_x - torque_z);
        self.motors[2].set_thrust(base + torque_x + torque_z);
        self.motors[3].set_thrust(base + torque_x - torque_z);
    }

    pub fn add_motor(&mut self, motor: Engine) {
        self.motors.push(motor);
    }

    pub fn set_motor(&mut self, collective_thrust: f32, desired_torque: Vector3<f32>) {
        self.collective_thrust = collective_thrust;
        self.desired_torque = desired_torque;
    }
}
