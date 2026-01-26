use nalgebra::{SVector, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct Halfspace3 {
    pub a: Vector3<f64>,
    pub b: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Halfspace4 {
    pub a: SVector<f64, 4>,
    pub b: f64,
}

pub fn project_qp3(
    x_nom: Vector3<f64>,
    box_min: Vector3<f64>,
    box_max: Vector3<f64>,
    constraints: &[Halfspace3],
    iters: usize,
) -> Vector3<f64> {
    let mut x = x_nom;
    let mut corr = vec![Vector3::new(0.0, 0.0, 0.0); constraints.len() + 1];
    for _ in 0..iters.max(1) {
        // Box projection.
        let y = x + corr[0];
        let x_new = clamp_vec3(y, box_min, box_max);
        corr[0] = y - x_new;
        x = x_new;

        // Halfspaces.
        for (idx, c) in constraints.iter().enumerate() {
            let slot = idx + 1;
            let y = x + corr[slot];
            let x_new = project_halfspace3(y, c.a, c.b);
            corr[slot] = y - x_new;
            x = x_new;
        }
    }
    x
}

pub fn project_qp4(
    x_nom: SVector<f64, 4>,
    box_min: SVector<f64, 4>,
    box_max: SVector<f64, 4>,
    constraints: &[Halfspace4],
    iters: usize,
) -> SVector<f64, 4> {
    let mut x = x_nom;
    let mut corr = vec![SVector::<f64, 4>::zeros(); constraints.len() + 1];
    for _ in 0..iters.max(1) {
        let y = x + corr[0];
        let x_new = clamp_vec4(y, box_min, box_max);
        corr[0] = y - x_new;
        x = x_new;

        for (idx, c) in constraints.iter().enumerate() {
            let slot = idx + 1;
            let y = x + corr[slot];
            let x_new = project_halfspace4(y, c.a, c.b);
            corr[slot] = y - x_new;
            x = x_new;
        }
    }
    x
}

fn project_halfspace3(y: Vector3<f64>, a: Vector3<f64>, b: f64) -> Vector3<f64> {
    let aa = a.dot(&a);
    if aa <= 1.0e-12 {
        return y;
    }
    let ay = a.dot(&y);
    if ay >= b {
        y
    } else {
        y + a * ((b - ay) / aa)
    }
}

fn project_halfspace4(y: SVector<f64, 4>, a: SVector<f64, 4>, b: f64) -> SVector<f64, 4> {
    let aa = a.dot(&a);
    if aa <= 1.0e-12 {
        return y;
    }
    let ay = a.dot(&y);
    if ay >= b {
        y
    } else {
        y + a * ((b - ay) / aa)
    }
}

fn clamp_vec3(v: Vector3<f64>, min: Vector3<f64>, max: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        v.x.max(min.x).min(max.x),
        v.y.max(min.y).min(max.y),
        v.z.max(min.z).min(max.z),
    )
}

fn clamp_vec4(v: SVector<f64, 4>, min: SVector<f64, 4>, max: SVector<f64, 4>) -> SVector<f64, 4> {
    let mut out = v;
    for i in 0..4 {
        out[i] = out[i].max(min[i]).min(max[i]);
    }
    out
}

