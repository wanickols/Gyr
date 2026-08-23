use nalgebra::{UnitQuaternion, Vector3};

pub struct Engine {
    pub position: Vector3<f32>,
    pub thrust: f32,
}

impl Engine {
    pub fn new(position: Vector3<f32>) -> Self {
        Engine {
            position,
            thrust: 0.0,
        }
    }

    pub fn torque(&self) -> Vector3<f32> {
        self.position.cross(&self.force())
    }

    pub fn set_thrust(&mut self, thrust: f32) {
        self.thrust = thrust;
    }

    //Private
    fn force(&self) -> Vector3<f32> {
        Vector3::new(0.0, self.thrust, 0.0)
    }
}

pub struct Drone {
    //Linear
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub mass: f32,

    //Rotational
    pub orientation: UnitQuaternion<f32>,
    pub angular_velocity: Vector3<f32>,
    pub inertia: Vector3<f32>,

    // Components
    pub engines: [Engine; 4],
}

impl Drone {
    pub fn new(position: Vector3<f32>, orientation: UnitQuaternion<f32>, mass: f32) -> Self {
        Drone {
            position,
            velocity: Vector3::zeros(),
            orientation,
            angular_velocity: Vector3::zeros(),
            mass,
            inertia: Vector3::new(1.0, 1.0, 1.0),
            engines: Drone::init_engines(),
        }
    }

    pub fn total_thrust(&self) -> Vector3<f32> {
        let thrust = self.engines.iter().map(|engine| engine.thrust).sum::<f32>();

        Vector3::new(0.0, thrust, 0.0)
    }

    pub fn total_torque(&self) -> Vector3<f32> {
        self.engines
            .iter()
            .map(|engine| engine.torque())
            .fold(Vector3::zeros(), |acc, torque| acc + torque)
    }

    pub fn set_thrust(&mut self, engine_index: usize, thrust: f32) {
        if engine_index < self.engines.len() {
            self.engines[engine_index].set_thrust(thrust);
        }
    }

    //Private
    fn init_engines() -> [Engine; 4] {
        [
            Engine::new(Vector3::new(-0.5, 0.0, 0.5)),
            Engine::new(Vector3::new(0.5, 0.0, 0.5)),
            Engine::new(Vector3::new(-0.5, 0.0, -0.5)),
            Engine::new(Vector3::new(0.5, 0.0, -0.5)),
        ]
    }
}
