use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use custom_collider::CustomSphere;
use parry3d::shape::SharedShape;

mod custom_collider;


fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Initialize Rapier 3D physics
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        // Optional: draws wireframe outlines around physics colliders
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, (setup_scene, setup_physics))
        .run();
}

/// Sets up the camera and lighting.
fn setup_scene(mut commands: Commands) {
    // Directional light with shadows
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // 3D Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 7.0, 16.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
    ));
}

/// Spawns the ground plane and the 4 falling balls.
fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // --- 1. Ground Plane ---
    let ground_size = 20.0;
    let ground_thickness = 0.2;

    commands.spawn((
        // Visual representation
        Mesh3d(meshes.add(Cuboid::new(ground_size, ground_thickness, ground_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::from_xyz(0.0, -ground_thickness / 2.0, 0.0),
        // Physics representation (Rapier cuboid takes half-extents)
        RigidBody::Fixed,
        Collider::cuboid(
            ground_size / 2.0,
            ground_thickness / 2.0,
            ground_size / 2.0,
        ),
    ));

    // --- 2. Four Falling Balls ---
    let ball_radius = 0.5;
    let ball_mesh = meshes.add(Sphere::new(ball_radius));

    // (Position, Color, Bounciness/Restitution)
    let balls = [
        (Vec3::new(-3.0, 5.0, 0.0), Color::srgb(0.9, 0.2, 0.2), 0.8),
        (Vec3::new(-1.0, 7.5, 0.5), Color::srgb(0.2, 0.4, 0.9), 0.7),
        (Vec3::new(1.0, 6.0, -0.5), Color::srgb(0.9, 0.8, 0.2), 0.85),
        (Vec3::new(3.0, 9.0, 0.0), Color::srgb(0.8, 0.3, 0.9), 0.6),
    ];

    for (position, color, bounciness) in balls {
        commands.spawn((
            // Visual representation
            Mesh3d(ball_mesh.clone()),
            MeshMaterial3d(materials.add(color)),
            Transform::from_translation(position),
            // Physics representation
            RigidBody::Dynamic,
            Restitution::coefficient(bounciness),

            //
            // <----- Switch between custom collider and ball collider here --->
            //
            //Collider::from(SharedShape::new(CustomSphere::new(ball_radius))),
            Collider::ball(ball_radius),
        ));
    }
}
