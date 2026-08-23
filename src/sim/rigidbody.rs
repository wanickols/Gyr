use crate::sim::physics;
use nalgebra::{UnitQuaternion, Vector3};

pub struct BodyId(pub usize);

pub struct RigidBody {
    //Linear
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub mass: f32,

    //Rotational
    pub orientation: UnitQuaternion<f32>,
    pub angular_velocity: Vector3<f32>,
    pub inertia: Vector3<f32>,

    force_accumulator: Vector3<f32>,
    torque_accumulator: Vector3<f32>,
}

impl RigidBody {
    pub fn new(position: Vector3<f32>, orientation: UnitQuaternion<f32>, mass: f32) -> Self {
        RigidBody {
            position,
            velocity: Vector3::zeros(),
            mass,
            orientation,
            angular_velocity: Vector3::zeros(),
            inertia: Vector3::new(1.0, 1.0, 1.0),
            force_accumulator: Vector3::zeros(),
            torque_accumulator: Vector3::zeros(),
        }
    }

    //Accumulation here incremented so the next step can apply the forces and torques to the rigid body.
    pub fn apply_force(&mut self, force: Vector3<f32>) {
        self.force_accumulator += force;
    }

    pub fn apply_torque(&mut self, torque: Vector3<f32>) {
        self.torque_accumulator += torque;
    }

    pub fn apply_force_at_point(&mut self, force: Vector3<f32>, point: Vector3<f32>) {
        self.force_accumulator += force;

        let r = point - self.position;
        self.torque_accumulator += r.cross(&force);
    }

    pub fn integrate(&mut self, dt: f32) {
        self.apply_linear(dt);
        self.apply_rotational(dt);
        // Clear accumulators after integration
        self.force_accumulator = Vector3::zeros();
        self.torque_accumulator = Vector3::zeros();
    }

    fn apply_linear(&mut self, dt: f32) {
        physics::apply_force(&mut self.velocity, self.force_accumulator, self.mass, dt);
        physics::apply_velocity(&mut self.position, self.velocity, dt);
    }

    fn apply_rotational(&mut self, dt: f32) {
        physics::apply_torque(
            &mut self.angular_velocity,
            self.torque_accumulator,
            self.inertia,
            dt,
        );
        physics::apply_angular_velocity(&mut self.orientation, self.angular_velocity, dt);
    }

    pub fn print_state(&self, time: f32) {
        let (roll, pitch, yaw) = self.orientation.euler_angles();

        println!(
            "t={:.0}s
  pos:      ({:>8.2}, {:>8.2}, {:>8.2})
  vel:      ({:>8.2}, {:>8.2}, {:>8.2})
  force:    ({:>8.2}, {:>8.2}, {:>8.2})
  torque:   ({:>8.2}, {:>8.2}, {:>8.2})
  ang vel:  ({:>8.2}, {:>8.2}, {:>8.2})
  euler:    ({:>8.2}°, {:>8.2}°, {:>8.2}°)
",
            time,
            self.position.x,
            self.position.y,
            self.position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
            self.force_accumulator.x,
            self.force_accumulator.y,
            self.force_accumulator.z,
            self.torque_accumulator.x,
            self.torque_accumulator.y,
            self.torque_accumulator.z,
            self.angular_velocity.x,
            self.angular_velocity.y,
            self.angular_velocity.z,
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees(),
        );
    }
}
