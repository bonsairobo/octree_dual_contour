use glam::{Mat2, Vec2, Vec3A, Vec3Swizzles};

pub fn sphere(r: f32, p: Vec3A) -> f32 {
    p.length() - r
}

pub fn plane(o: Vec3A, n: Vec3A, p: Vec3A) -> f32 {
    (p - o).dot(n)
}

pub fn torus(t: Vec2, p: Vec3A) -> f32 {
    let q = Vec2::new(p.xz().length() - t.x, p.y);
    q.length() - t.y
}

pub fn cube(b: Vec3A, p: Vec3A) -> f32 {
    let q = p.abs() - b;
    q.max(Vec3A::ZERO).length() + q.max_element().min(0.0)
}

pub fn octahedron(p: Vec3A, s: f32) -> f32 {
    let p = p.abs();
    (p.x + p.y + p.z - s) * 0.57735027
}

// c is the sin/cos of the angle
pub fn solid_angle(p: Vec3A, c: Vec2, ra: f32) -> f32 {
    let q = Vec2::new(p.xz().length(), p.y);
    let l = q.length() - ra;
    let m = (q - c * q.dot(c).clamp(0.0, ra)).length();
    l.max(m * (c.y * q.x - c.x * q.y).signum())
}

pub fn capped_cylinder(p: Vec3A, h: f32, r: f32) -> f32 {
    let d = Vec2::new(p.xz().length(), p.y).abs() - Vec2::new(h, r);
    d.x.max(d.y).min(0.0) + d.max(Vec2::ZERO).length()
}

pub fn tri_prism(p: Vec3A, h: Vec2) -> f32 {
    let q = p.abs();
    (q.z - h.y).max((q.x * 0.866025 + p.y * 0.5).max(-p.y) - h.x * 0.5)
}

pub fn twist(primitive: impl Fn(Vec3A) -> f32, p: Vec3A, k: f32) -> f32 {
    let c = (k * p.y).cos();
    let s = (k * p.y).sin();
    let m = Mat2::from_cols(Vec2::new(c, -s), Vec2::new(s, c));
    let q2 = m * p.xz();
    let q = Vec3A::new(q2.x, q2.y, p.y);
    primitive(q)
}
