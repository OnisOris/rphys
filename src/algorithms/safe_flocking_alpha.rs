use crate::algorithms::flocking::{
    DEFAULT_BOUNDARY_RADIUS, DEFAULT_BOUNDARY_WEIGHT, DEFAULT_MAX_FORCE, DEFAULT_MAX_SPEED,
    DEFAULT_SPEED_LIMIT,
};
use crate::algorithms::obstacles::{paper_obstacles, ObstaclePoly};
use crate::algorithms::qp_project::{project_qp4, Halfspace4};
use crate::models::particles::ParticleModel;
use nalgebra::{SVector, Vector3};
use serde::{Deserialize, Serialize};

pub const DEFAULT_SAFE_ALPHA_NEIGHBOR_RADIUS: f64 = 2.6;
pub const DEFAULT_SAFE_ALPHA_DESIRED_DISTANCE: f64 = 1.4;
pub const DEFAULT_SAFE_ALPHA_SIGMA_EPS: f64 = 0.1;
pub const DEFAULT_SAFE_ALPHA_BUMP_H: f64 = 0.2;
pub const DEFAULT_SAFE_ALPHA_PHI_A: f64 = 5.0;
pub const DEFAULT_SAFE_ALPHA_PHI_B: f64 = 5.0;
pub const DEFAULT_SAFE_ALPHA_WEIGHT: f64 = 1.0;
pub const DEFAULT_SAFE_ALPHA_ALIGNMENT_WEIGHT: f64 = 0.65;

const DEFAULT_QP_ITERS: usize = 14;
const DEFAULT_EPS: f64 = 1.0e-2;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct SafeFlockAlphaParams {
    // Nominal alpha-lattice.
    pub neighbor_radius: f64,
    pub desired_distance: f64,
    pub sigma_eps: f64,
    pub bump_h: f64,
    pub phi_a: f64,
    pub phi_b: f64,
    pub alpha_weight: f64,
    pub alignment_weight: f64,
    pub boundary_radius: f64,
    pub boundary_weight: f64,
    pub max_speed: f64,
    pub max_force: f64,
    pub speed_limit: f64,

    // Safety (CBF-QP filter).
    pub use_obstacles: bool,
    pub use_agent_cbf: bool,
    pub agent_safe_distance: f64,
    pub cbf_neighbor_radius: f64,

    pub lambda1: f64,
    pub lambda2: f64,
    pub delta_theta: f64,
    pub delta2_star: f64,
    pub use_moving_obstacle_terms: bool,
    pub two_pass: bool,

    // Control bounds on u (acceleration).
    pub u_min: [f64; 3],
    pub u_max: [f64; 3],

    // Slack for feasibility (a·u + s >= b, s >= 0).
    pub slack_weight: f64,
    pub slack_max: f64,

    pub smooth_eps: f64,
    pub qp_iters: usize,

    pub obstacles: Vec<ObstaclePoly>,
}

impl Default for SafeFlockAlphaParams {
    fn default() -> Self {
        Self {
            neighbor_radius: DEFAULT_SAFE_ALPHA_NEIGHBOR_RADIUS,
            desired_distance: DEFAULT_SAFE_ALPHA_DESIRED_DISTANCE,
            sigma_eps: DEFAULT_SAFE_ALPHA_SIGMA_EPS,
            bump_h: DEFAULT_SAFE_ALPHA_BUMP_H,
            phi_a: DEFAULT_SAFE_ALPHA_PHI_A,
            phi_b: DEFAULT_SAFE_ALPHA_PHI_B,
            alpha_weight: DEFAULT_SAFE_ALPHA_WEIGHT,
            alignment_weight: DEFAULT_SAFE_ALPHA_ALIGNMENT_WEIGHT,
            boundary_radius: DEFAULT_BOUNDARY_RADIUS,
            boundary_weight: DEFAULT_BOUNDARY_WEIGHT,
            max_speed: DEFAULT_MAX_SPEED,
            max_force: DEFAULT_MAX_FORCE,
            speed_limit: DEFAULT_SPEED_LIMIT,
            use_obstacles: true,
            use_agent_cbf: true,
            agent_safe_distance: 0.9,
            cbf_neighbor_radius: 2.6,
            lambda1: 2.0,
            lambda2: 2.0,
            delta_theta: 0.2,
            delta2_star: 0.0,
            use_moving_obstacle_terms: true,
            two_pass: false,
            u_min: [-6.0, -6.0, -6.0],
            u_max: [6.0, 6.0, 6.0],
            slack_weight: 50.0,
            slack_max: 50.0,
            smooth_eps: DEFAULT_EPS,
            qp_iters: DEFAULT_QP_ITERS,
            obstacles: paper_obstacles(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SafeFlockingAlpha {
    pub params: SafeFlockAlphaParams,
    state: SafeFlockAlphaState,
}

#[derive(Debug, Clone, Default)]
struct SafeFlockAlphaState {
    u_nom: Vec<Vector3<f64>>,
    u_safe: Vec<Vector3<f64>>,
    slack: Vec<f64>,
    active: Vec<f64>,
    total: Vec<f64>,
}

impl SafeFlockingAlpha {
    pub fn new(params: SafeFlockAlphaParams) -> Self {
        Self {
            params,
            state: SafeFlockAlphaState::default(),
        }
    }

    pub fn debug_flat(&self) -> Vec<f32> {
        let n = self.state.u_safe.len();
        let mut out = Vec::with_capacity(n * 9);
        for i in 0..n {
            let un = self.state.u_nom[i];
            let us = self.state.u_safe[i];
            out.push(un.x as f32);
            out.push(un.y as f32);
            out.push(un.z as f32);
            out.push(us.x as f32);
            out.push(us.y as f32);
            out.push(us.z as f32);
            out.push(self.state.slack[i] as f32);
            out.push(self.state.active[i] as f32);
            out.push(self.state.total[i] as f32);
        }
        out
    }

    pub fn apply(&mut self, model: &mut ParticleModel, plane_2d: bool) {
        let n = model.len();
        if n == 0 {
            return;
        }
        let dt = model.dt();
        if !dt.is_finite() || dt <= 0.0 {
            return;
        }
        let t = model.time();

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

        self.ensure_state(n);

        let masses = model.masses().to_vec();
        let drags = model.drags().to_vec();

        // Precompute nominal u_n for everyone (needed for agent-agent CBF as neighbor prediction).
        self.compute_nominal(&positions, &velocities, plane_2d);

        // Neighbor acceleration prediction for inter-agent CBF:
        // use last applied safe u (more consistent than u_nom when filter is active).
        // At t=0 there is no previous safe input yet, so start from u_nom.
        let mut u_pred = if t <= 0.0 {
            self.state.u_nom.clone()
        } else {
            self.state.u_safe.clone()
        };

        let passes = if self.params.two_pass { 2 } else { 1 };

        let mut u_next = vec![Vector3::new(0.0, 0.0, 0.0); n];
        let mut slack_next = vec![0.0; n];
        let mut active_next = vec![0.0; n];
        let mut total_next = vec![0.0; n];

        for pass in 0..passes {
            for i in 0..n {
                let p_i = positions[i];
                let v_i = velocities[i];
                let mass = masses.get(i).copied().unwrap_or(1.0).max(1.0e-6);
                let drag = drags.get(i).copied().unwrap_or(0.0);
                let gamma_i = -drag / mass;

                let u_nom = self.state.u_nom[i];
                let (u_safe, slack, active, total) = self.filter_u(
                    i,
                    t,
                    &positions,
                    &velocities,
                    &masses,
                    &drags,
                    p_i,
                    v_i,
                    gamma_i,
                    u_nom,
                    &u_pred,
                );

                let mut u_cmd = u_safe;
                if plane_2d {
                    u_cmd.z = 0.0;
                }

                u_next[i] = u_cmd;
                slack_next[i] = slack;
                active_next[i] = active;
                total_next[i] = total;
            }

            if pass + 1 < passes {
                u_pred.clone_from(&u_next);
            }
        }

        self.state.u_safe.clone_from(&u_next);
        self.state.slack.clone_from(&slack_next);
        self.state.active.clone_from(&active_next);
        self.state.total.clone_from(&total_next);

        for i in 0..n {
            let mass = masses.get(i).copied().unwrap_or(1.0).max(1.0e-6);
            model.set_force(i, self.state.u_safe[i] * mass);
        }
    }

    fn ensure_state(&mut self, n: usize) {
        if self.state.u_nom.len() == n {
            return;
        }
        self.state.u_nom = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.u_safe = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.slack = vec![0.0; n];
        self.state.active = vec![0.0; n];
        self.state.total = vec![0.0; n];
    }

    fn compute_nominal(&mut self, positions: &[Vector3<f64>], velocities: &[Vector3<f64>], plane_2d: bool) {
        let n = positions.len();
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
            h = DEFAULT_SAFE_ALPHA_BUMP_H;
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
            DEFAULT_SAFE_ALPHA_PHI_A
        };
        let b = if self.params.phi_b.is_finite() && self.params.phi_b > 0.0 {
            self.params.phi_b
        } else {
            DEFAULT_SAFE_ALPHA_PHI_B
        };

        let d_alpha = sigma_norm_scalar(d.max(0.0), eps);
        let r_alpha = sigma_norm_scalar(r, eps);

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
                let diff = positions[j] - pos_i;
                let dist2 = diff.norm_squared();
                if dist2 > neighbor_r2 {
                    continue;
                }
                neighbors += 1;

                let denom = (1.0 + eps * dist2).sqrt();
                let z = (denom - 1.0) / eps;
                let nij = if denom > 0.0 { diff / denom } else { diff };

                let varphi = phi_alpha(z, d_alpha, r_alpha, h, a, b);
                grad_sum += nij * varphi;

                let aij = bump_rho(if r_alpha > 0.0 { z / r_alpha } else { 0.0 }, h);
                cons_sum += (velocities[j] - vel_i) * aij;
            }

            if neighbors > 0 {
                cons_sum *= 1.0 / neighbors as f64;
            }

            let mut u = grad_sum * self.params.alpha_weight + cons_sum * self.params.alignment_weight;

            // Soft boundary centered at origin.
            let dist = pos_i.norm();
            if dist > self.params.boundary_radius && dist > 0.0 {
                let dir = pos_i / dist;
                u += -dir * (dist - self.params.boundary_radius) * self.params.boundary_weight;
            }

            // Speed limiting (damping when above max_speed).
            let speed = vel_i.norm();
            if speed > self.params.max_speed && speed > 0.0 {
                let dir = vel_i / speed;
                u += -dir * (speed - self.params.max_speed) * self.params.speed_limit;
            }

            // Clamp nominal magnitude.
            let mag = u.norm();
            if mag > self.params.max_force && mag > 0.0 {
                u = u / mag * self.params.max_force;
            }

            if plane_2d {
                u.z = 0.0;
            }
            self.state.u_nom[i] = u;
        }
    }

    fn filter_u(
        &self,
        i: usize,
        t: f64,
        positions: &[Vector3<f64>],
        velocities: &[Vector3<f64>],
        masses: &[f64],
        drags: &[f64],
        p_i: Vector3<f64>,
        v_i: Vector3<f64>,
        gamma_i: f64,
        u_nom: Vector3<f64>,
        u_pred: &[Vector3<f64>],
    ) -> (Vector3<f64>, f64, f64, f64) {
        let mut constraints: Vec<Halfspace4> = Vec::new();

        let lambda1 = self.params.lambda1.max(0.0);
        let lambda2 = self.params.lambda2.max(0.0);
        let pi1 = lambda1 * lambda2;
        let pi2 = lambda1 + lambda2;

        let sigma = self.params.slack_weight.max(1.0e-6).sqrt();
        let slack_coef = 1.0 / sigma;

        if self.params.use_obstacles {
            for ob in &self.params.obstacles {
                let p_ob = ob.pos(t);
                let r = p_i - p_ob;
                let r2 = r.norm_squared();
                if r2 < 1.0e-10 {
                    continue;
                }

                let h = r2 - ob.d * ob.d;
                let (c_base, lfh) = if self.params.use_moving_obstacle_terms {
                    let v_ob = ob.vel(t);
                    let a_ob = ob.acc();
                    let v_rel = v_i - v_ob;
                    let lfh = 2.0 * r.dot(&v_rel);
                    let c = 2.0 * v_rel.dot(&v_rel) + 2.0 * r.dot(&(gamma_i * v_i - a_ob));
                    (c, lfh)
                } else {
                    let lfh = 2.0 * r.dot(&v_i);
                    let c = 2.0 * v_i.dot(&v_i) + 2.0 * r.dot(&(gamma_i * v_i));
                    (c, lfh)
                };

                let delta1 = 2.0 * r.norm() * self.params.delta_theta.max(0.0);
                let phi = pi1 * h + pi2 * lfh - delta1;
                let xi1 = 1.0 + self.params.delta2_star;
                let xi2 = 1.0 - self.params.delta2_star;

                let a_u = r * (2.0 * xi1);
                let b = -phi - xi1 * c_base;
                constraints.push(Halfspace4 {
                    a: SVector::<f64, 4>::new(a_u.x, a_u.y, a_u.z, slack_coef),
                    b,
                });

                if self.params.delta2_star > 0.0 {
                    let a_u = r * (2.0 * xi2);
                    let b = -phi - xi2 * c_base;
                    constraints.push(Halfspace4 {
                        a: SVector::<f64, 4>::new(a_u.x, a_u.y, a_u.z, slack_coef),
                        b,
                    });
                }
            }
        }

        if self.params.use_agent_cbf && positions.len() == velocities.len() {
            let n = positions.len();
            let mut rr = self.params.cbf_neighbor_radius;
            if !rr.is_finite() || rr <= 0.0 {
                rr = self.params.neighbor_radius.max(0.0);
            }
            let rr2 = rr * rr;
            let d_safe2 = self.params.agent_safe_distance.max(0.0).powi(2);

            for j in 0..n {
                if j == i {
                    continue;
                }
                let p_j = positions[j];
                let r = p_i - p_j;
                let r2 = r.norm_squared();
                if r2 <= 1.0e-10 || r2 > rr2 {
                    continue;
                }
                let v_j = velocities[j];
                let v_rel = v_i - v_j;

                let mass_j = masses.get(j).copied().unwrap_or(1.0).max(1.0e-6);
                let drag_j = drags.get(j).copied().unwrap_or(0.0);
                let gamma_j = -drag_j / mass_j;

                // Neighbor predicted acceleration (use last safe input snapshot).
                let u_j = u_pred.get(j).copied().unwrap_or(self.state.u_nom[j]);

                let h = r2 - d_safe2;
                let lfh = 2.0 * r.dot(&v_rel);
                let c = 2.0 * v_rel.dot(&v_rel)
                    + 2.0 * r.dot(&(gamma_i * v_i - gamma_j * v_j - u_j));

                let b = -(c + pi1 * h + pi2 * lfh);
                let a_u = r * 2.0;
                constraints.push(Halfspace4 {
                    a: SVector::<f64, 4>::new(a_u.x, a_u.y, a_u.z, slack_coef),
                    b,
                });
            }
        }

        let y_nom = SVector::<f64, 4>::new(u_nom.x, u_nom.y, u_nom.z, 0.0);
        let umin = vec3_from(self.params.u_min);
        let umax = vec3_from(self.params.u_max);
        let box_min = SVector::<f64, 4>::new(umin.x, umin.y, umin.z, 0.0);
        let box_max = SVector::<f64, 4>::new(umax.x, umax.y, umax.z, sigma * self.params.slack_max.max(0.0));

        let y = project_qp4(y_nom, box_min, box_max, &constraints, self.params.qp_iters);
        let u = Vector3::new(y[0], y[1], y[2]);
        let slack = (y[3] / sigma).max(0.0);

        let mut active = 0usize;
        let tol = 1.0e-3;
        for c in &constraints {
            let margin = c.a.dot(&y) - c.b;
            if margin <= tol {
                active += 1;
            }
        }

        (u, slack, active as f64, constraints.len() as f64)
    }
}

fn vec3_from(v: [f64; 3]) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
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
