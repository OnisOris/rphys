use crate::algorithms::flocking::{
    DEFAULT_BOUNDARY_RADIUS, DEFAULT_BOUNDARY_WEIGHT, DEFAULT_MAX_FORCE, DEFAULT_MAX_SPEED,
    DEFAULT_SPEED_LIMIT,
};
use crate::models::particles::ParticleModel;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

pub const DEFAULT_ALPHA_NEIGHBOR_RADIUS: f64 = 2.6;
pub const DEFAULT_ALPHA_DESIRED_DISTANCE: f64 = 1.4;
pub const DEFAULT_ALPHA_SIGMA_EPS: f64 = 0.1;
pub const DEFAULT_ALPHA_BUMP_H: f64 = 0.2;
pub const DEFAULT_ALPHA_PHI_A: f64 = 5.0;
pub const DEFAULT_ALPHA_PHI_B: f64 = 5.0;
pub const DEFAULT_ALPHA_WEIGHT: f64 = 1.0;
pub const DEFAULT_ALPHA_ALIGNMENT_WEIGHT: f64 = 0.65;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct FlockAlphaParams {
    /// Interaction radius r (m). Neighbors beyond r do not contribute.
    pub neighbor_radius: f64,
    /// Target spacing d (m) for the alpha-lattice term.
    pub desired_distance: f64,

    /// Smoothing epsilon for the sigma-norm.
    pub sigma_eps: f64,
    /// Bump-function parameter h in [0,1). Higher values mean smoother cutoff near r.
    pub bump_h: f64,
    /// Uneven-sigmoid parameters (a,b). If a==b, it becomes symmetric.
    pub phi_a: f64,
    pub phi_b: f64,

    /// Scaling for the lattice-shaping (gradient) term.
    pub alpha_weight: f64,
    /// Scaling for the velocity consensus term.
    pub alignment_weight: f64,

    /// Soft boundary centered at origin (m).
    pub boundary_radius: f64,
    pub boundary_weight: f64,

    /// Speed limiting.
    pub max_speed: f64,
    pub max_force: f64,
    pub speed_limit: f64,
}

impl Default for FlockAlphaParams {
    fn default() -> Self {
        Self {
            neighbor_radius: DEFAULT_ALPHA_NEIGHBOR_RADIUS,
            desired_distance: DEFAULT_ALPHA_DESIRED_DISTANCE,
            sigma_eps: DEFAULT_ALPHA_SIGMA_EPS,
            bump_h: DEFAULT_ALPHA_BUMP_H,
            phi_a: DEFAULT_ALPHA_PHI_A,
            phi_b: DEFAULT_ALPHA_PHI_B,
            alpha_weight: DEFAULT_ALPHA_WEIGHT,
            alignment_weight: DEFAULT_ALPHA_ALIGNMENT_WEIGHT,
            boundary_radius: DEFAULT_BOUNDARY_RADIUS,
            boundary_weight: DEFAULT_BOUNDARY_WEIGHT,
            max_speed: DEFAULT_MAX_SPEED,
            max_force: DEFAULT_MAX_FORCE,
            speed_limit: DEFAULT_SPEED_LIMIT,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FlockingAlpha {
    pub params: FlockAlphaParams,
}

impl FlockingAlpha {
    pub fn new(params: FlockAlphaParams) -> Self {
        Self { params }
    }

    /// Apply a simplified Olfati-Saber alpha-lattice shaping term + velocity consensus.
    pub fn apply(&self, model: &mut ParticleModel, plane_2d: bool) {
        let mut positions = model.positions().to_vec();
        let mut velocities = model.velocities().to_vec();
        if plane_2d {
            for p in positions.iter_mut() {
                p.z = 0.0;
            }
            for v in velocities.iter_mut() {
                v.z = 0.0;
            }
        }

        let n = positions.len();
        if n == 0 {
            return;
        }

        let mut r = self.params.neighbor_radius;
        let d = self.params.desired_distance;
        if !r.is_finite() || r <= 0.0 {
            r = d.max(0.0);
        }
        if r < d {
            r = d;
        }
        let neighbor_r2 = r * r;

        let mut eps = self.params.sigma_eps;
        if !eps.is_finite() || eps <= 0.0 {
            eps = 1.0e-9;
        }

        let mut h = self.params.bump_h;
        if !h.is_finite() {
            h = DEFAULT_ALPHA_BUMP_H;
        }
        if h < 0.0 {
            h = 0.0;
        }
        if h >= 1.0 {
            h = 0.999;
        }

        let a = if self.params.phi_a.is_finite() && self.params.phi_a > 0.0 {
            self.params.phi_a
        } else {
            DEFAULT_ALPHA_PHI_A
        };
        let b = if self.params.phi_b.is_finite() && self.params.phi_b > 0.0 {
            self.params.phi_b
        } else {
            DEFAULT_ALPHA_PHI_B
        };

        let d_alpha = sigma_norm_scalar(d.max(0.0), eps);
        let r_alpha = sigma_norm_scalar(r, eps);

        let mut forces = Vec::with_capacity(n);
        for i in 0..n {
            let pos_i = positions[i];
            let vel_i = velocities[i];

            let mut grad_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut cons_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut neighbors = 0usize;

            for j in 0..n {
                if i == j {
                    continue;
                }
                let diff = positions[j] - pos_i; // q_j - q_i
                let dist2 = diff.norm_squared();
                if dist2 > neighbor_r2 {
                    continue;
                }
                neighbors += 1;

                let denom = (1.0 + eps * dist2).sqrt();
                let z = (denom - 1.0) / eps; // ||q_j - q_i||_sigma
                let nij = if denom > 0.0 { diff / denom } else { diff };

                let varphi = phi_alpha(z, d_alpha, r_alpha, h, a, b);
                grad_sum += nij * varphi;

                let aij = bump_rho(if r_alpha > 0.0 { z / r_alpha } else { 0.0 }, h);
                cons_sum += (velocities[j] - vel_i) * aij;
            }

            if neighbors > 0 {
                cons_sum *= 1.0 / neighbors as f64;
            }

            let mut force = grad_sum * self.params.alpha_weight + cons_sum * self.params.alignment_weight;

            // Soft boundary centered at origin.
            let dist = pos_i.norm();
            if dist > self.params.boundary_radius && dist > 0.0 {
                let dir = pos_i / dist;
                force += -dir * (dist - self.params.boundary_radius) * self.params.boundary_weight;
            }

            // Speed limiting (damping when above max_speed).
            let speed = vel_i.norm();
            if speed > self.params.max_speed && speed > 0.0 {
                let dir = vel_i / speed;
                force += -dir * (speed - self.params.max_speed) * self.params.speed_limit;
            }

            // Clamp force magnitude.
            let fmag = force.norm();
            if fmag > self.params.max_force && fmag > 0.0 {
                force = force / fmag * self.params.max_force;
            }

            if plane_2d {
                force.z = 0.0;
            }
            forces.push(force);
        }

        for (i, f) in forces.into_iter().enumerate() {
            model.set_force(i, f);
        }
    }
}

fn sigma_norm_scalar(value: f64, eps: f64) -> f64 {
    ((1.0 + eps * value * value).sqrt() - 1.0) / eps
}

fn bump_rho(s: f64, h: f64) -> f64 {
    if s < 0.0 {
        0.0
    } else if s < h {
        1.0
    } else if s <= 1.0 {
        let denom = 1.0 - h;
        if denom <= 0.0 {
            0.0
        } else {
            0.5 * (1.0 + (std::f64::consts::PI * (s - h) / denom).cos())
        }
    } else {
        0.0
    }
}

fn sigma1(x: f64) -> f64 {
    x / (1.0 + x * x).sqrt()
}

fn uneven_phi(x: f64, a: f64, b: f64) -> f64 {
    let denom = (4.0 * a * b).sqrt();
    let c = if denom > 0.0 { (a - b).abs() / denom } else { 0.0 };
    0.5 * ((a + b) * sigma1(x + c) + (a - b))
}

fn phi_alpha(z: f64, d_alpha: f64, r_alpha: f64, h: f64, a: f64, b: f64) -> f64 {
    if r_alpha <= 0.0 {
        return 0.0;
    }
    bump_rho(z / r_alpha, h) * uneven_phi(z - d_alpha, a, b)
}

