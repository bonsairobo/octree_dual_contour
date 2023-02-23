use crate::qef::Qef;
use glam::Vec3A;
use ilattice::prelude::Extent;

/// May have false negatives, but never has false positives.
pub fn branch_empty_check(extent_diagonal: f32, samples: &[f32; 8]) -> bool {
    // HACK: This padding helps with slight non-Euclidean warping of space.
    const PAD_FACTOR: f32 = 1.5;
    let padded_diag = PAD_FACTOR * extent_diagonal;
    for &d in samples {
        if d.abs() > padded_diag {
            // One way to know for sure that the isosurface does not
            // intersect this cell is if any of the vertex samples indicate
            // the surface is farther away than the max distance between
            // vertices.
            return true;
        }
    }
    // Even a homogeneous cell may intersect the surface after further
    // subdivision.
    false
}

pub fn cell_is_bipolar(samples: &[f32; 8]) -> bool {
    let mut any_negative = false;
    let mut any_positive = false;
    for &sample in samples {
        any_negative |= sample < 0.0;
        any_positive |= sample >= 0.0;
    }
    any_negative && any_positive
}

pub fn central_gradient(sdf: impl Fn(Vec3A) -> f32, p: Vec3A, delta: f32) -> Vec3A {
    let h = 0.5 * delta;
    let dx = Vec3A::new(h, 0.0, 0.0);
    let dy = Vec3A::new(0.0, h, 0.0);
    let dz = Vec3A::new(0.0, 0.0, h);
    Vec3A::new(
        sdf(p + dx) - sdf(p - dx),
        sdf(p + dy) - sdf(p - dy),
        sdf(p + dz) - sdf(p - dz),
    ) / h
}

/// Calculates the average of all edge intersections (each estimated with
/// linear interpolation).
pub fn estimate_interior_vertex(extent: &Extent<Vec3A>, samples: &[f32; 8]) -> Vec3A {
    let corners = extent.corners3();
    let mut estimate = Vec3A::ZERO;
    let mut num_bipolar_edges = 0.0;
    for [e1, e2] in Extent::<Vec3A>::EDGES3 {
        let s1 = samples[e1];
        let s2 = samples[e2];
        if (s1 < 0.0) != (s2 < 0.0) {
            num_bipolar_edges += 1.0;
            // Lerp the edge vertices.
            let diff = s2 - s1;
            let s1_lerp = s2 / diff;
            let s2_lerp = -s1 / diff;
            estimate += s1_lerp * corners[e1] + s2_lerp * corners[e2];
        }
    }
    estimate / num_bipolar_edges
}

pub fn estimate_interior_vertex_qef(
    extent: &Extent<Vec3A>,
    samples: &[f32; 8],
    sdf: impl Fn(Vec3A) -> f32,
    precision: f32,
) -> (Qef, Qef) {
    let mut regularized_qef = Qef::default();
    let mut exact_qef = Qef::default();

    let corners = extent.corners3();
    for [e1, e2] in Extent::<Vec3A>::EDGES3 {
        let s1 = samples[e1];
        let s2 = samples[e2];
        if (s1 < 0.0) != (s2 < 0.0) {
            // Lerp the edge vertices.
            let diff = s2 - s1;
            let s1_lerp = s2 / diff;
            let s2_lerp = -s1 / diff;
            let edge_cross_p = s1_lerp * corners[e1] + s2_lerp * corners[e2];

            // Central differencing around the edge crossing.
            // Needs 6 samples.
            let normal = central_gradient(&sdf, edge_cross_p, 0.0001).normalize();

            regularized_qef = regularized_qef
                + Qef::isometric_probabilistic_plane(
                    edge_cross_p,
                    normal,
                    precision * extent.shape.x,
                    precision,
                );

            exact_qef = exact_qef + Qef::plane(edge_cross_p, normal);
        }
    }

    (regularized_qef, exact_qef)
}
