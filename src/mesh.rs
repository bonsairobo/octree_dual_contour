use glam::Vec3A;

pub type MeshVertexId = u32;
pub const NULL_MESH_VERTEX_ID: MeshVertexId = MeshVertexId::MAX;

/// Repair normals for vertices on sharp edges.
///
/// This may add vertices to the mesh in order to allow multiple normals at the
/// same position.
pub fn repair_sharp_normals(
    normal_similarity_threshold: f32,
    tri_indices: &mut [u32],
    positions: &mut Vec<Vec3A>,
    normals: &mut Vec<Vec3A>,
) {
    for t in tri_indices.chunks_exact_mut(3) {
        let mut tri = [t[0], t[1], t[2]];
        let n = tri.map(|v| normals[v as usize]);
        let p = tri.map(|v| positions[v as usize]);

        let tri_normal = (p[1] - p[0]).cross(p[2] - p[0]).normalize();

        // Force dissident normals to use the triangle's normal.
        for ti in 0..3 {
            if n[ti].dot(tri_normal) < normal_similarity_threshold {
                let new_vert = positions.len() as MeshVertexId;
                positions.push(p[ti]);
                normals.push(tri_normal);
                tri[ti] = new_vert;
            }
        }

        t.copy_from_slice(&tri);
    }
}
