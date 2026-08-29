use bevy::prelude::*;
use gyr_sim::Simulation;

#[derive(Resource)]
struct SimResource {
    sim: Simulation,
}

#[derive(Component)]
struct DroneVisual;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimResource {
            sim: Simulation::new(),
        })
        .add_systems(Startup, setup_bevy)
        .add_systems(Startup, setup_sim)
        .add_systems(Update, update_drone)
        .run();
}

fn setup_bevy(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20.0, 20.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
    ));

    // The extremely advanced drone
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 0.25, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        Transform::from_xyz(0.0, 2.0, 0.0),
        DroneVisual,
    ));

    // Light
    commands.spawn((
        PointLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 20.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_sim(mut sim: ResMut<SimResource>) {
    sim.sim.set_altitude_with_default_ori(5.0);
}

fn update_drone(
    mut sim: ResMut<SimResource>,
    mut drone_query: Query<&mut Transform, With<DroneVisual>>,
) {
    sim.sim.step(0.01);

    let state = sim.sim.drone_state();
    let mut transform = drone_query.single_mut().unwrap();

    transform.translation = Vec3::new(state.position.x, state.position.y, state.position.z);

    let q = state.orientation.quaternion();

    transform.rotation = Quat::from_xyzw(q.i, q.j, q.k, q.w);
}
