//! Octree Dual Contouring
//!
//! # References
//!
//! - Tao Ju, Frank Losasso, Scott Schaefer, Joe Warren ["Dual Contouring of
//!   Hermite Data"](https://www.cs.rice.edu/~jwarren/papers/dualcontour.pdf)
//! - Philip Trettner, Leif Kobbelt ["Fast and Robust QEF Minimization using
//!   Probabilistic
//!   Quadrics"](https://www.graphics.rwth-aachen.de/publication/03308/)
//!     - [Reference
//!       implementation](https://github.com/Philip-Trettner/probabilistic-quadrics)
//!
//! # Project Status
//!
//! This is currently just a prototype for understanding the limitations of this
//! technique. My current assessment:
//!
//! ## Pros
//!
//! - can reproduce sharp features from hermite data
//! - built-in octree simplification via QEF
//!
//! ## Cons
//!
//! - requires parameter tuning to avoid artifacts
//! - probably slow? (still need to benchmark)

mod cell_octree;
mod contour_octree;
mod mesh;
mod qef;
mod sdf;
mod tables;

pub mod sdf_primitives;

pub use cell_octree::*;
pub use mesh::*;
pub use sdf::*;
