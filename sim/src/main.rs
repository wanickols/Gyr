use workspace::sim::Simulation;

fn main() {
    let mut sim = Simulation::new();

    sim.sim_drone(10.0, 0.01);
}
