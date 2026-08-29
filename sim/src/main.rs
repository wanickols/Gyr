use gyr_sim::Simulation;

fn main() {
    let mut sim = Simulation::new();

    sim.test(10.0, 0.01);
}
