use bevy::{
    pbr::wireframe::{Wireframe, WireframePlugin},
    prelude::*,
    render::{mesh::Indices, render_resource::PrimitiveTopology},
};
use glam::{Vec3, Vec3A};
use ilattice::prelude::Extent;
use octree_dual_contour::{
    central_gradient, repair_sharp_normals, sdf_primitives::*, CellOctree, MeshVertexId,
};
use smooth_bevy_cameras::{controllers::fps::*, LookTransformPlugin};
use std::time::Instant;

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(WireframePlugin)
        .add_plugin(LookTransformPlugin)
        .add_plugin(FpsCameraPlugin::default())
        .add_startup_system(spawn_scene)
        .run();
}

fn spawn_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let red_material = materials.add(Color::RED.into());

    // let field = |p: Vec3A| torus(Vec2::new(4.0, 2.0), p);
    // let field = |p: Vec3A| {
    //     torus(Vec2::new(4.0, 2.0), p)
    //         .max(plane(Vec3A::splat(0.0), Vec3A::splat(1.0).normalize(), p))
    //         .max(-torus(Vec2::new(4.0, 1.5), p))
    // };
    let field = |p: Vec3A| {
        cube(Vec3A::splat(8.0), p)
            .max(-tri_prism(p, Vec2::new(10.0, 1.0)))
            .max(-torus(Vec2::new(4.0, 2.0), p - Vec3A::splat(5.0)))
            .max(-octahedron(p + Vec3A::splat(3.5), 4.20))
    };
    // let field = |p: Vec3A| {
    //     solid_angle(
    //         p,
    //         Vec2::new(
    //             std::f32::consts::FRAC_PI_3.sin(),
    //             std::f32::consts::FRAC_PI_3.cos(),
    //         ),
    //         10.0,
    //     )
    //     .max(plane(
    //         Vec3A::splat(0.0),
    //         Vec3A::new(1.0, 0.0, 0.0).normalize(),
    //         p,
    //     ))
    // };
    // let field = |p: Vec3A| capped_cylinder(p, 8.0, 8.0);
    // let field = |p: Vec3A| tri_prism(p, Vec2::new(8.0, 1.0));
    // let field = |p: Vec3A| {
    //     (octahedron(p, 4.20))
    //         .max(-sphere(2.0, p - Vec3A::new(0.0, 3.0, 0.0)))
    //         .max(-sphere(2.0, p - Vec3A::new(3.0, 0.0, 0.0)))
    //         .max(-sphere(2.0, p - Vec3A::new(-3.0, 0.0, 0.0)))
    //         .max(-sphere(2.0, p - Vec3A::new(0.0, 0.0, 3.0)))
    //         .max(-sphere(2.0, p - Vec3A::new(0.0, 0.0, -3.0)))
    //         .max(-sphere(2.0, p - Vec3A::new(0.0, 0.0, 0.0)))
    // };

    // QEF and branch_empty_check are very unstable for twists because they create
    // non-Euclidean 3D space embeddings. We at least need QEF-adaptive sampling
    // to handle these.
    //
    // let field = |p: Vec3A| twist(|q| torus(Vec2::new(4.0, 2.0), q), p, 160.0);
    // let field = |p: Vec3A| twist(|q| cube(Vec3A::splat(3.0), q), p, 50.0);

    // Create an isosurface mesh.
    let root_cell = Extent::<Vec3A>::from_min_and_lub([-10.0; 3].into(), [10.0; 3].into());
    let max_depth = 7;
    let error_tolerance = 0.00001;
    let precision = 0.1;
    let build_t0 = Instant::now();
    let mut octree =
        CellOctree::build(root_cell, max_depth, error_tolerance, precision, field).unwrap();
    println!("octree build took {} us", build_t0.elapsed().as_micros());
    let mut min_leaf_depth = u8::MAX;
    let mut max_leaf_depth = 0;
    let mut positions: Vec<Vec3A> = Vec::new();
    let mut normals = Vec::new();
    let mut quad_indices = Vec::new();
    let mut tri_indices = Vec::new();
    let contour_t0 = Instant::now();
    octree.dual_contour(
        |_cell_id, cell| {
            min_leaf_depth = min_leaf_depth.min(cell.depth);
            max_leaf_depth = max_leaf_depth.max(cell.depth);

            cell.mesh_vertex_id = positions.len() as MeshVertexId;
            positions.push(cell.vertex_estimate.into());
            normals.push(central_gradient(&field, cell.vertex_estimate.into(), 0.001).normalize());
        },
        |q| {
            quad_indices.extend_from_slice(&[q[0], q[2], q[1], q[1], q[2], q[3]]);
        },
        |tri| {
            tri_indices.extend_from_slice(&tri);
        },
    );
    println!("dual contour took {} us", contour_t0.elapsed().as_micros());
    println!("depth = {min_leaf_depth}..={max_leaf_depth}");
    tri_indices.append(&mut quad_indices);

    // Now we need to create the mesh by copying the proper vertices out of the
    // octree. Since not all vertices will be used, we need to recreate the
    // vertex IDs based on the new mesh.
    let all_cells = octree.all_cells();
    let mut tri_indices: Vec<_> = tri_indices
        .into_iter()
        .map(|i| all_cells[i as usize].mesh_vertex_id)
        .collect();

    repair_sharp_normals(0.95, &mut tri_indices, &mut positions, &mut normals);

    println!("# isosurface vertices = {}", positions.len());
    println!("# isosurface triangles = {}", tri_indices.len() / 3);

    let mut isomesh = Mesh::new(PrimitiveTopology::TriangleList);
    isomesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.clone());
    isomesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals.clone());
    isomesh.set_indices(Some(Indices::U32(tri_indices)));
    let isomesh = meshes.add(isomesh);
    commands
        .spawn(PbrBundle {
            material: red_material.clone(),
            mesh: isomesh,
            ..Default::default()
        })
        .insert(Wireframe);

    // Draw a line at every surface normal.
    // let mut line_verts = Vec::new();
    // for (pos, normal) in positions.iter().zip(normals.iter()) {
    //     line_verts.push(*pos);
    //     line_verts.push(*pos + *normal / 3.0);
    // }
    // let mut line_mesh = Mesh::new(PrimitiveTopology::LineList);
    // line_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, line_verts);
    // let line_mesh = meshes.add(line_mesh);
    // commands.spawn(PbrBundle {
    //     material: red_material.clone(),
    //     mesh: line_mesh,
    //     ..Default::default()
    // });

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 100_000.0,
            range: 1000.,
            shadows_enabled: false,
            ..default()
        },
        transform: Transform::from_xyz(50.0, 50.0, 50.0),
        ..default()
    });

    commands
        .spawn(Camera3dBundle::default())
        .insert(FpsCameraBundle::new(
            FpsCameraController {
                translate_sensitivity: 8.0,
                ..Default::default()
            },
            Vec3::new(12.0, 12.0, 12.0),
            Vec3::new(0., 0., 0.),
        ));
}
