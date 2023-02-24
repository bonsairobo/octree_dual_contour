/// Pairs of octants (o0, o1) where o0 and o1 are face-adjacent.
pub const FACE_ADJACENT_OCTANTS: [[[u8; 2]; 4]; 3] = [
    [
        //  -X     +X
        [0b000, 0b001],
        [0b010, 0b011],
        [0b100, 0b101],
        [0b110, 0b111],
    ],
    [
        //  -Y     +Y
        [0b000, 0b010],
        [0b001, 0b011],
        [0b100, 0b110],
        [0b101, 0b111],
    ],
    [
        //  -Z     +Z
        [0b000, 0b100],
        [0b001, 0b101],
        [0b010, 0b110],
        [0b011, 0b111],
    ],
];

// Quartets of octants (o0, o1, o2, o3) where all of o0-o3 are edge-adjacent.
// Ordering of octants in a quartet is consistently Z order.
pub const EDGE_ADJACENT_OCTANTS_NEG_X: [u8; 4] = [0b000, 0b010, 0b100, 0b110];
pub const EDGE_ADJACENT_OCTANTS_POS_X: [u8; 4] = [0b001, 0b011, 0b101, 0b111];
pub const EDGE_ADJACENT_OCTANTS_NEG_Y: [u8; 4] = [0b000, 0b100, 0b001, 0b101];
pub const EDGE_ADJACENT_OCTANTS_POS_Y: [u8; 4] = [0b010, 0b110, 0b011, 0b111];
pub const EDGE_ADJACENT_OCTANTS_NEG_Z: [u8; 4] = [0b000, 0b001, 0b010, 0b011];
pub const EDGE_ADJACENT_OCTANTS_POS_Z: [u8; 4] = [0b100, 0b101, 0b110, 0b111];

pub const EDGE_ADJACENT_OCTANTS: [[[u8; 4]; 2]; 3] = [
    [EDGE_ADJACENT_OCTANTS_NEG_X, EDGE_ADJACENT_OCTANTS_POS_X],
    [EDGE_ADJACENT_OCTANTS_NEG_Y, EDGE_ADJACENT_OCTANTS_POS_Y],
    [EDGE_ADJACENT_OCTANTS_NEG_Z, EDGE_ADJACENT_OCTANTS_POS_Z],
];

pub const FACE_TO_EDGE_ADJACENT_OCTANTS: [[[u8; 4]; 4]; 3] = [
    [
        // X
        EDGE_ADJACENT_OCTANTS_NEG_Y,
        EDGE_ADJACENT_OCTANTS_POS_Y,
        EDGE_ADJACENT_OCTANTS_NEG_Z,
        EDGE_ADJACENT_OCTANTS_POS_Z,
    ],
    [
        // Y
        EDGE_ADJACENT_OCTANTS_NEG_Z,
        EDGE_ADJACENT_OCTANTS_POS_Z,
        EDGE_ADJACENT_OCTANTS_NEG_X,
        EDGE_ADJACENT_OCTANTS_POS_X,
    ],
    [
        // Z
        EDGE_ADJACENT_OCTANTS_NEG_X,
        EDGE_ADJACENT_OCTANTS_POS_X,
        EDGE_ADJACENT_OCTANTS_NEG_Y,
        EDGE_ADJACENT_OCTANTS_POS_Y,
    ],
];
pub const FACE_TO_EDGE_AXIS: [[usize; 4]; 3] = [[1, 1, 2, 2], [2, 2, 0, 0], [0, 0, 1, 1]];
pub const FACE_TO_EDGE_NODE_ORDERS: [[usize; 4]; 4] =
    [[0, 0, 1, 1], [0, 0, 1, 1], [0, 1, 0, 1], [0, 1, 0, 1]];
pub const FACE_TO_EDGE_MIRRORS: [[usize; 4]; 4] =
    [[2, 3, 0, 1], [2, 3, 0, 1], [1, 0, 3, 2], [1, 0, 3, 2]];
