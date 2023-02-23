use glam::Vec3A;
use std::ops::Add;

/// (Probabilistic) Quadric Error Function
///
/// `x^T A x - 2 b^T x + c`
///
/// Uses the probabilistic QEF solver from "Fast and Robust QEF Minimization
/// using Probabilistic Quadrics" by Trettner and Kobbelt. This formulation of
/// the quadric is guaranteed nonsingular and robust to discrete sampling noise.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Qef {
    a00: f32,
    a01: f32,
    a02: f32,
    a11: f32,
    a12: f32,
    a22: f32,

    b: Vec3A,

    c: f32,
}

impl Qef {
    pub fn from_coefficients(a_cols: [[f32; 3]; 3], b: Vec3A, c: f32) -> Self {
        Self {
            // Keep one triangle of the symmetric matrix.
            a00: a_cols[0][0],
            a01: a_cols[0][1],
            a02: a_cols[0][2],
            a11: a_cols[1][1],
            a12: a_cols[1][2],
            a22: a_cols[2][2],
            b,
            c,
        }
    }

    /// Residual L2 error. `x^T A x - 2 b^T x + c`
    pub fn error(&self, p: Vec3A) -> f32 {
        // Ax
        let ax = Vec3A::new(
            self.a00 * p[0] + self.a01 * p[1] + self.a02 * p[2],
            self.a01 * p[0] + self.a11 * p[1] + self.a12 * p[2],
            self.a02 * p[0] + self.a12 * p[1] + self.a22 * p[2],
        );

        p.dot(ax) - 2.0 * p.dot(self.b) + self.c
    }

    pub fn minimizer(&self) -> Vec3A {
        let a = self.a00;
        let b = self.a01;
        let c = self.a02;
        let d = self.a11;
        let e = self.a12;
        let f = self.a22;

        let ad = a * d;
        let ae = a * e;
        let af = a * f;
        let bc = b * c;
        let be = b * e;
        let bf = b * f;
        let df = d * f;
        let ce = c * e;
        let cd = c * d;

        let be_cd = be - cd;
        let bc_ae = bc - ae;
        let ce_bf = ce - bf;

        let denom = 1.0 / (a * df + 2.0 * b * ce - ae * e - bf * b - cd * c);

        let nom0 = self.b.dot(Vec3A::new(df - e * e, ce_bf, be_cd));
        let nom1 = self.b.dot(Vec3A::new(ce_bf, af - c * c, bc_ae));
        let nom2 = self.b.dot(Vec3A::new(be_cd, bc_ae, ad - b * b));

        denom * Vec3A::new(nom0, nom1, nom2)
    }

    pub fn plane(p: Vec3A, n: Vec3A) -> Self {
        let d = p.dot(n);
        Self::from_coefficients(self_outer_product(n.into()), d * n, d * d)
    }

    pub fn isometric_probabilistic_plane(
        mean_p: Vec3A,
        mean_n: Vec3A,
        stddev_p: f32,
        stddev_n: f32,
    ) -> Self {
        let sp2 = stddev_p * stddev_p;
        let sn2 = stddev_n * stddev_n;
        let d = mean_p.dot(mean_n);

        let mut a = self_outer_product(mean_n.to_array());
        a[0][0] += sn2;
        a[1][1] += sn2;
        a[2][2] += sn2;

        let b = mean_n * d + mean_p * sn2;
        let c = d * d + sn2 * mean_p.dot(mean_p) + sp2 * mean_n.dot(mean_n) + 3.0 * sp2 * sn2;

        Self::from_coefficients(a, b, c)
    }
}

impl Add for Qef {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            a00: self.a00 + rhs.a00,
            a01: self.a01 + rhs.a01,
            a02: self.a02 + rhs.a02,
            a11: self.a11 + rhs.a11,
            a12: self.a12 + rhs.a12,
            a22: self.a22 + rhs.a22,
            b: self.b + rhs.b,
            c: self.c + rhs.c,
        }
    }
}

fn self_outer_product([a, b, c]: [f32; 3]) -> [[f32; 3]; 3] {
    [
        [a * a, a * b, a * c],
        [a * b, b * b, b * c],
        [a * c, b * c, c * c],
    ]
}
