use crate::{tables::*, Cell, CellId, CellOctree, Edge, Face};

impl CellOctree {
    pub fn dual_contour(
        &mut self,
        mut visit_leaf_cell: impl FnMut(CellId, &mut Cell),
        mut visit_quad: impl FnMut([CellId; 4]),
        mut visit_triangle: impl FnMut([CellId; 3]),
    ) {
        self.clear_stacks();

        // The general strategy for isosurface extraction via dual contouring is
        // to visit all bipolar edges on the interior of the root cell and
        // create an isosurface quad or triangle for each edge to pass through.
        // These isosurface facets connect vertices at the center of each cell
        // sharing the bipolar edge.
        //
        // The non-recursive algorithm requires 3 stacks that are consumed in
        // this order. One downside is that the face and edge stacks may get
        // very large while traversing all cells first.
        //
        // PERF: we might get away with starting to consume faces and edges
        // before the cells are complete

        // Start from the root again to seed contouring.
        self.cell_stack.push(self.root_id);
        while let Some(cell) = self.cell_stack.pop() {
            contour_cell_interior(self, cell, &mut visit_leaf_cell);
        }

        // Search for interior faces.
        while let Some(face) = self.face_stack.pop() {
            contour_face_interior(self, face);
        }

        // Search for interior edges.
        while let Some(edge) = self.edge_stack.pop() {
            contour_edge_interior(self, edge, &mut visit_quad, &mut visit_triangle);
        }
    }
}

// 8 cells, 12 faces, 6 edges
#[inline]
fn contour_cell_interior(
    octree: &mut CellOctree,
    cell_id: CellId,
    visit_leaf_cell: &mut impl FnMut(CellId, &mut Cell),
) {
    let cell = &mut octree.all_cells[cell_id as usize];

    if cell.is_leaf {
        // If we've reached the desired level of detail, run a user-defined
        // interior contouring routine.
        visit_leaf_cell(cell_id, cell);
        return;
    }

    // Recursively identify all bipolar edges on the interior of the parent cell.

    for &child in cell.children.iter().flatten() {
        octree.cell_stack.push(child);
    }

    // What remains of the interior of the parent cell can be found entirely
    // in the face interiors and edge interiors of the descendant cells.
    for axis in 0..3 {
        for face in 0..4 {
            let face_cell_ids =
                FACE_ADJACENT_OCTANTS[axis][face].map(|o| cell.children[o as usize]);
            if let [Some(f0), Some(f1)] = face_cell_ids {
                octree.face_stack.push(Face {
                    axis,
                    cells: [f0, f1],
                });
            }
        }

        for edge in 0..2 {
            // Because we just began partitioning this cell, we know there
            // are 4 children intersecting each edge.
            let edge_cells = EDGE_ADJACENT_OCTANTS[axis][edge].map(|o| cell.children[o as usize]);
            if let [Some(e0), Some(e1), Some(e2), Some(e3)] = edge_cells {
                octree.edge_stack.push(Edge {
                    axis,
                    cells: [e0, e1, e2, e3],
                    is_duplicate: [false; 4],
                });
            }
        }
    }
}

// 4 faces and 4 edges
#[inline]
fn contour_face_interior(octree: &mut CellOctree, face: Face) {
    // PRECONDITION: `face` cells are given in increasing order (- side of face to + side).

    let face_cells = face.cells.map(|i| &octree.all_cells[i as usize]);

    if face_cells[0].is_leaf && face_cells[1].is_leaf {
        // No edges on the face interior.
        return;
    }

    // Split both sides.
    //
    // Even though we may end up duplicating leaves, we will filter the
    // resulting degenerate triangles later.
    //
    // In all cases, mirror the octants because they have different parents:
    //
    // ```
    // +--+--+--+--+
    // |  | x|x |  |
    // +--+--+--+--+
    // |  | x|x |  |
    // +--+--+--+--+
    //   f0    f1
    // ```

    let mut get_child_cell_id = |parent: CellId, octant: u8| -> (Option<CellId>, bool) {
        let cell = &mut octree.all_cells[parent as usize];
        if cell.is_leaf {
            // Continue participation. Note that we may cause a duplicate on an
            // edge.
            (Some(parent), true)
        } else {
            (cell.children[octant as usize], false)
        }
    };

    for face_i in 0..4 {
        let o = FACE_ADJACENT_OCTANTS[face.axis][face_i];

        // Mirror permutation is independent of which face we're looking at.
        if let [Some(f0), Some(f1)] = [
            get_child_cell_id(face.cells[0], o[1]).0,
            get_child_cell_id(face.cells[1], o[0]).0,
        ] {
            octree.face_stack.push(Face {
                axis: face.axis,
                cells: [f0, f1],
            });
        }
    }

    for edge_i in 0..4 {
        let o = FACE_TO_EDGE_ADJACENT_OCTANTS[face.axis][edge_i];

        // Depending on the orientation of the Z curve relative to the parent
        // octants...
        //
        //      +---+   +---+      +---+   +---+
        //      | x-|---|-x |      | x |   | x |
        //      |   | / |   |  vs  | | | \ | | |
        //      | x-|---|-x |      | x |   | x |
        //      +---+   +---+      +---+   +---+
        //       f0      f1         f0      f1
        //
        // ...we have to change the order in which we:
        //
        // 1. select from the parent nodes.
        let order = FACE_TO_EDGE_NODE_ORDERS[edge_i];
        // 2. mirror across the face.
        let mirror = FACE_TO_EDGE_MIRRORS[edge_i];

        let mut child_is_dup = [false; 4];
        let next_edge = [0, 1, 2, 3].map(|i| {
            let (child_id, is_dup) = get_child_cell_id(face.cells[order[i]], o[mirror[i]]);
            child_is_dup[i] = is_dup;
            child_id
        });

        // "Face axis" is not the same as "edge axis."
        let edge_axis = FACE_TO_EDGE_AXIS[face.axis][edge_i];

        if let [Some(e0), Some(e1), Some(e2), Some(e3)] = next_edge {
            octree.edge_stack.push(Edge {
                axis: edge_axis,
                cells: [e0, e1, e2, e3],
                is_duplicate: child_is_dup,
            });
        }
    }
}

// 2 edges
#[inline]
fn contour_edge_interior(
    octree: &mut CellOctree,
    edge: Edge,
    visit_quad: &mut impl FnMut([CellId; 4]),
    visit_triangle: &mut impl FnMut([CellId; 3]),
) {
    // PRECONDITION: `edge` nodes are given in Z order.
    //
    //     e0 e1    00 01
    //     e2 e3    10 11

    let axis = edge.axis;
    let edge_cells = edge.cells.map(|i| &octree.all_cells[i as usize]);
    if edge_cells.iter().all(|c| c.is_leaf) {
        visit_leaf_edge(edge, edge_cells, axis, visit_quad, visit_triangle);
    } else {
        // We must continue bisecting this edge.
        let mut get_child_cell_id = |parent: CellId, octant: u8| -> Option<CellId> {
            let cell = &mut octree.all_cells[parent as usize];
            if cell.is_leaf {
                // Continue participation.
                Some(parent)
            } else {
                cell.children[octant as usize]
            }
        };
        for edge_i in 0..2 {
            let o = EDGE_ADJACENT_OCTANTS[axis][edge_i];

            // Swap diagonal octants because they have different parents:
            //
            // ```
            // +--+--+  +--+--+
            // |  |  |  |  |  |
            // +--+--+  +--+--+
            // |  | x|  |x |  |
            // +--+--+  +--+--+
            //    e0       e1
            //
            // +--+--+  +--+--+
            // |  | x|  |x |  |
            // +--+--+  +--+--+
            // |  |  |  |  |  |
            // +--+--+  +--+--+
            //    e2       e3
            // ```
            let next_edge = [0, 1, 2, 3].map(|i| get_child_cell_id(edge.cells[i], o[3 - i]));

            if let [Some(e0), Some(e1), Some(e2), Some(e3)] = next_edge {
                octree.edge_stack.push(Edge {
                    axis,
                    cells: [e0, e1, e2, e3],
                    is_duplicate: [false; 4],
                });
            }
        }
    }
}

fn visit_leaf_edge(
    edge: Edge,
    edge_cells: [&Cell; 4],
    axis: usize,
    visit_quad: &mut impl FnMut([CellId; 4]),
    visit_triangle: &mut impl FnMut([CellId; 3]),
) {
    // Check if this leaf edge is bipolar. We can just check the samples on
    // the smallest cell.
    let mut min_cell = 0;
    let mut max_depth = 0;
    for (i, cell) in edge_cells.iter().enumerate() {
        if cell.depth > max_depth {
            max_depth = cell.depth;
            min_cell = i;
        }
    }
    // Select the edge at the opposite corner of the octant.
    let octants = EDGE_ADJACENT_OCTANTS[axis];
    let opposite_corner = [3, 2, 1, 0][min_cell];
    let c0 = octants[0][opposite_corner];
    let c1 = octants[1][opposite_corner];
    let sdf = &edge_cells[min_cell].samples;
    let d0 = sdf[c0 as usize];
    let d1 = sdf[c1 as usize];

    let flip = match (d0 < 0.0, d1 < 0.0) {
        (true, false) => true,
        (false, true) => false,
        _ => return, // Not a bipolar edge.
    };

    // Filter triangles with duplicate vertices (from edges with duplicate
    // cells). Because the triangles must share a diagonal, we know a
    // duplicate can't occur in both triangles. We also know that if any
    // duplicate exists, it will necessarily appear twice around this edge.
    let tris = [[0, 2, 1], [1, 2, 3]];
    let first_tri_num_dups = tris[0]
        .iter()
        .map(|&t| edge.is_duplicate[t] as u8)
        .sum::<u8>();
    if first_tri_num_dups > 0 {
        // Skip the degenerate triangle.
        let use_tri = if first_tri_num_dups == 1 {
            tris[0]
        } else {
            tris[1]
        };
        if flip {
            let flipped_tri = [use_tri[0], use_tri[2], use_tri[1]];
            visit_triangle(flipped_tri.map(|i| edge.cells[i]));
        } else {
            visit_triangle(use_tri.map(|i| edge.cells[i]));
        }
    } else {
        // No degenerate triangles found.
        if flip {
            visit_quad([edge.cells[2], edge.cells[3], edge.cells[0], edge.cells[1]]);
        } else {
            visit_quad(edge.cells);
        }
    }
}
