use crate::models::particles::ParticleModel;
use crate::algorithms::obstacles::{paper_obstacles, ObstaclePoly};
use crate::algorithms::qp_project::{project_qp3, Halfspace3};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

const DEFAULT_QP_ITERS: usize = 12;
const DEFAULT_EPS: f64 = 1.0e-2;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum LeaderTrajectory {
    Paper,
    Static { position: [f64; 3] },
    Poly { a2: [f64; 3], a1: [f64; 3], a0: [f64; 3] },
    Circle { center: [f64; 3], radius: f64, omega: f64 },
    Custom,
}

impl LeaderTrajectory {
    fn state(&self, t: f64) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        match self {
            LeaderTrajectory::Paper => {
                let omega = -0.06;
                let phase = std::f64::consts::PI;
                let angle = omega * t + phase;
                let c = angle.cos();
                let s = angle.sin();
                let p = Vector3::new(60.0 + 25.0 * c, 60.0 + 25.0 * s, 0.5 * t);
                let v = Vector3::new(1.5 * s, -1.5 * c, 0.5);
                let a = Vector3::new(-0.09 * c, -0.09 * s, 0.0);
                (p, v, a)
            }
            LeaderTrajectory::Static { position } => {
                let p = Vector3::new(position[0], position[1], position[2]);
                (p, Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0))
            }
            LeaderTrajectory::Poly { a2, a1, a0 } => {
                let p = Vector3::new(
                    a2[0] * t * t + a1[0] * t + a0[0],
                    a2[1] * t * t + a1[1] * t + a0[1],
                    a2[2] * t * t + a1[2] * t + a0[2],
                );
                let v = Vector3::new(
                    2.0 * a2[0] * t + a1[0],
                    2.0 * a2[1] * t + a1[1],
                    2.0 * a2[2] * t + a1[2],
                );
                let a = Vector3::new(2.0 * a2[0], 2.0 * a2[1], 2.0 * a2[2]);
                (p, v, a)
            }
            LeaderTrajectory::Circle { center, radius, omega } => {
                let angle = omega * t;
                let c = angle.cos();
                let s = angle.sin();
                let p = Vector3::new(
                    center[0] + radius * c,
                    center[1] + radius * s,
                    center[2],
                );
                let v = Vector3::new(-radius * omega * s, radius * omega * c, 0.0);
                let a = Vector3::new(-radius * omega * omega * c, -radius * omega * omega * s, 0.0);
                (p, v, a)
            }
            LeaderTrajectory::Custom => custom_leader_trajectory(t),
        }
    }
}

fn custom_leader_trajectory(t: f64) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    // Пользовательская траектория лидера (меняй тут).
    // Возвращает (p, v, a) в зависимости от времени t.
    let radius = 15.0;
    let omega = 0.04;
    let angle = omega * t;
    let p = Vector3::new(radius * angle.cos(), radius * angle.sin(), 0.2 * t);
    let v = Vector3::new(-radius * omega * angle.sin(), radius * omega * angle.cos(), 0.2);
    let a = Vector3::new(
        -radius * omega * omega * angle.cos(),
        -radius * omega * omega * angle.sin(),
        0.0,
    );
    (p, v, a)
}

// ObstaclePoly lives in algorithms/obstacles.rs

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct FormationEcbfParams {
    // Nominal formation control
    pub k1: f64,
    pub k2: f64,
    pub gamma1: f64,
    pub gamma2: f64,
    pub m1: f64,
    pub m2: f64,

    // Distributed observer
    pub obs_k1: f64,
    pub obs_k2: f64,
    pub obs_k3: f64,
    pub obs_a1: f64,
    pub obs_a2: f64,
    pub obs_b1: f64,
    pub obs_b2: f64,

    // Disturbance observer
    pub do_kappa1: f64,
    pub do_kappa2: f64,
    pub do_kappa3: f64,
    pub do_eta1: f64,
    pub do_eta2: f64,
    pub do_eta3: f64,
    pub do_n1: f64,
    pub do_n2: f64,

    // Robust ECBF
    pub delta_theta: f64,
    pub delta2_star: f64,
    pub lambda1: f64,
    pub lambda2: f64,

    // Gravity + yaw (for attitude visualization)
    pub gravity: f64,
    pub desired_yaw: f64,
    pub leader_time_scale: f64,
    pub leader_paused: bool,

    // Smoothing / filters
    pub smooth_eps: f64,
    pub mu_dot_filter: f64,
    pub alpha_dot_filter: f64,

    // QP limits
    pub u_min: [f64; 3],
    pub u_max: [f64; 3],
    pub qp_iters: usize,

    // Obstacles and formation definition
    pub obstacles: Vec<ObstaclePoly>,
    pub formation_offsets: Vec<[f64; 3]>,
    pub auto_offsets: bool,

    // Graph (followers-only) adjacency and leader links
    pub adjacency: Vec<Vec<f64>>,
    pub leader_links: Vec<f64>,

    // Leader trajectory
    pub leader: LeaderTrajectory,

    // Moving obstacle terms in L_f h
    pub use_moving_obstacle_terms: bool,
}

impl Default for FormationEcbfParams {
    fn default() -> Self {
        Self {
            k1: 2.0,
            k2: 3.0,
            gamma1: 0.5,
            gamma2: 0.5,
            m1: 1.0,
            m2: 2.0,
            obs_k1: 1.0,
            obs_k2: 1.0,
            obs_k3: 2.0,
            obs_a1: 2.0,
            obs_a2: 1.0,
            obs_b1: 1.0,
            obs_b2: 2.0,
            do_kappa1: 4.0,
            do_kappa2: 2.0,
            do_kappa3: 3.0,
            do_eta1: 1.5,
            do_eta2: 1.5,
            do_eta3: 0.5,
            do_n1: 0.5,
            do_n2: 1.5,
            delta_theta: 0.2,
            delta2_star: 0.0,
            lambda1: 2.0,
            lambda2: 2.0,
            gravity: 9.81,
            desired_yaw: 0.0,
            leader_time_scale: 1.0,
            leader_paused: false,
            smooth_eps: DEFAULT_EPS,
            mu_dot_filter: 0.8,
            alpha_dot_filter: 0.8,
            u_min: [-6.0, -6.0, 0.0],
            u_max: [6.0, 6.0, 20.0],
            qp_iters: DEFAULT_QP_ITERS,
            obstacles: paper_obstacles(),
            formation_offsets: vec![
                [-3.0, 3.0, 0.0],
                [-3.0, -3.0, 0.0],
                [3.0, 3.0, 0.0],
                [3.0, -3.0, 0.0],
            ],
            auto_offsets: true,
            adjacency: vec![
                vec![0.0, 0.0, 1.0, 0.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![1.0, 1.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0, 0.0],
            ],
            leader_links: vec![1.0, 1.0, 0.0, 0.0],
            leader: LeaderTrajectory::Circle {
                center: [0.0, 0.0, 0.0],
                radius: 6.0,
                omega: 0.2,
            },
            use_moving_obstacle_terms: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FormationEcbf {
    pub params: FormationEcbfParams,
    state: FormationEcbfState,
}

#[derive(Debug, Clone, Default)]
struct FormationEcbfState {
    chi: Vec<Vector3<f64>>,
    varsigma: Vec<Vector3<f64>>,
    v_hat: Vec<Vector3<f64>>,
    theta_hat: Vec<Vector3<f64>>,
    p_state: Vec<Vector3<f64>>,
    mu_prev: Vec<Vector3<f64>>,
    mu_dot_prev: Vec<Vector3<f64>>,
    alpha_prev: Vec<Vector3<f64>>,
    alpha_dot_prev: Vec<Vector3<f64>>,
    mu_ready: Vec<bool>,
    alpha_ready: Vec<bool>,
    formation_offsets: Vec<Vector3<f64>>,
    offsets_ready: bool,
    attitudes: Vec<Vector3<f64>>,
    thrusts: Vec<f64>,
    leader_hold: Option<(Vector3<f64>, Vector3<f64>, Vector3<f64>)>,
}

impl FormationEcbf {
    pub fn new(params: FormationEcbfParams) -> Self {
        Self {
            params,
            state: FormationEcbfState::default(),
        }
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

        let (p0, v0, _a0) = self.leader_state(t);
        self.ensure_state(n, &positions, &velocities, p0, v0);
        self.ensure_offsets(&positions, p0);
        self.update_distributed_observer(n, dt, p0, v0);

        let masses = model.masses().to_vec();
        let drags = model.drags().to_vec();

        for i in 0..n {
            let p = positions[i];
            let v = velocities[i];
            let mass = masses.get(i).copied().unwrap_or(1.0).max(1.0e-6);
            let drag = drags.get(i).copied().unwrap_or(0.0);
            let gamma = -drag / mass;
            let gamma_v = v * gamma;

            let p_star = self.formation_offset(i);
            let z1 = p - self.state.chi[i] - p_star;
            let alpha = self.compute_alpha(z1, self.state.varsigma[i]);
            let alpha_dot = self.update_alpha_dot(i, alpha, dt);
            let z2 = v - alpha;

            let u_nom = -self.params.k2 * z2
                - z1
                - self.params.gamma1 * sig_pow_vec(z2, self.a1(), self.params.smooth_eps)
                - self.params.gamma2 * sig_pow_vec(z2, self.a2(), self.params.smooth_eps)
                + alpha_dot
                - self.state.theta_hat[i]
                - gamma_v;

            let constraints = self.build_constraints(
                p,
                v,
                gamma,
                &self.state.theta_hat[i],
                t,
            );

            let mut u = project_qp3(
                u_nom,
                vec3_from(self.params.u_min),
                vec3_from(self.params.u_max),
                &constraints,
                self.params.qp_iters,
            );

            if plane_2d {
                u.z = 0.0;
            }

            self.update_attitude(i, u, mass);
            model.set_force(i, u * mass);
            self.update_disturbance_observer(i, v, u, gamma_v, dt);
        }
    }

    fn ensure_state(
        &mut self,
        n: usize,
        _positions: &[Vector3<f64>],
        velocities: &[Vector3<f64>],
        p0: Vector3<f64>,
        v0: Vector3<f64>,
    ) {
        if self.state.chi.len() == n {
            return;
        }
        self.state.chi = vec![p0; n];
        self.state.varsigma = vec![v0; n];
        self.state.v_hat = velocities.to_vec();
        self.state.theta_hat = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.p_state = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.mu_prev = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.mu_dot_prev = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.alpha_prev = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.alpha_dot_prev = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.mu_ready = vec![false; n];
        self.state.alpha_ready = vec![false; n];
        self.state.formation_offsets = Vec::new();
        self.state.offsets_ready = false;
        self.state.attitudes = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.thrusts = vec![0.0; n];
        self.state.leader_hold = None;
    }

    fn ensure_offsets(&mut self, positions: &[Vector3<f64>], leader_pos: Vector3<f64>) {
        if self.state.offsets_ready {
            return;
        }
        let n = positions.len();
        if self.params.formation_offsets.len() == n {
            self.state.formation_offsets = self
                .params
                .formation_offsets
                .iter()
                .map(|v| vec3_from(*v))
                .collect();
            self.state.offsets_ready = true;
            return;
        }
        if self.params.auto_offsets {
            self.state.formation_offsets = positions
                .iter()
                .map(|p| *p - leader_pos)
                .collect();
            self.state.offsets_ready = true;
            return;
        }
        self.state.formation_offsets = vec![Vector3::new(0.0, 0.0, 0.0); n];
        self.state.offsets_ready = true;
    }

    fn update_distributed_observer(&mut self, n: usize, dt: f64, p0: Vector3<f64>, v0: Vector3<f64>) {
        let a1 = self.params.obs_a1 / self.params.obs_a2.max(1.0e-6);
        let b1 = self.params.obs_b1 / self.params.obs_b2.max(1.0e-6);
        let eps = self.params.smooth_eps;
        for i in 0..n {
            let mut sum_chi = Vector3::new(0.0, 0.0, 0.0);
            let mut sum_var = Vector3::new(0.0, 0.0, 0.0);
            for j in 0..n {
                if i == j {
                    continue;
                }
                let aij = self.adj_weight(i, j, n);
                if aij == 0.0 {
                    continue;
                }
                sum_chi += (self.state.chi[i] - self.state.chi[j]) * aij;
                sum_var += (self.state.varsigma[i] - self.state.varsigma[j]) * aij;
            }
            let a_i0 = self.leader_link(i, n);
            if a_i0 != 0.0 {
                sum_chi += (self.state.chi[i] - p0) * a_i0;
                sum_var += (self.state.varsigma[i] - v0) * a_i0;
            }

            let chi_dot = -self.params.obs_k1 * sig_pow_vec(sum_chi, a1, eps)
                - self.params.obs_k2 * sig_pow_vec(sum_chi, b1, eps)
                + self.state.varsigma[i];
            let varsigma_dot = -self.params.obs_k1 * sig_pow_vec(sum_var, a1, eps)
                - self.params.obs_k2 * sig_pow_vec(sum_var, b1, eps)
                - self.params.obs_k3 * sign_vec(sum_var, eps);

            self.state.chi[i] += chi_dot * dt;
            self.state.varsigma[i] += varsigma_dot * dt;
        }
    }

    fn compute_alpha(&self, z1: Vector3<f64>, varsigma: Vector3<f64>) -> Vector3<f64> {
        -self.params.k1 * z1
            - self.params.gamma1 * sig_pow_vec(z1, self.a1(), self.params.smooth_eps)
            - self.params.gamma2 * sig_pow_vec(z1, self.a2(), self.params.smooth_eps)
            + varsigma
    }

    fn update_alpha_dot(&mut self, i: usize, alpha: Vector3<f64>, dt: f64) -> Vector3<f64> {
        if !self.state.alpha_ready[i] {
            self.state.alpha_prev[i] = alpha;
            self.state.alpha_dot_prev[i] = Vector3::new(0.0, 0.0, 0.0);
            self.state.alpha_ready[i] = true;
            return Vector3::new(0.0, 0.0, 0.0);
        }
        let raw = (alpha - self.state.alpha_prev[i]) / dt;
        let beta = self.params.alpha_dot_filter.clamp(0.0, 0.999);
        let filtered = self.state.alpha_dot_prev[i] * beta + raw * (1.0 - beta);
        self.state.alpha_prev[i] = alpha;
        self.state.alpha_dot_prev[i] = filtered;
        filtered
    }

    fn update_disturbance_observer(
        &mut self,
        i: usize,
        v: Vector3<f64>,
        u: Vector3<f64>,
        gamma_v: Vector3<f64>,
        dt: f64,
    ) {
        let v_hat_dot = u + gamma_v + self.state.theta_hat[i];
        self.state.v_hat[i] += v_hat_dot * dt;
        let mu = v - self.state.v_hat[i];
        if !self.state.mu_ready[i] {
            self.state.mu_prev[i] = mu;
            self.state.mu_dot_prev[i] = Vector3::new(0.0, 0.0, 0.0);
            self.state.mu_ready[i] = true;
            return;
        }

        let raw_mu_dot = (mu - self.state.mu_prev[i]) / dt;
        let beta = self.params.mu_dot_filter.clamp(0.0, 0.999);
        let mu_dot = self.state.mu_dot_prev[i] * beta + raw_mu_dot * (1.0 - beta);

        let s = mu_dot + self.params.do_kappa1 * self.state.p_state[i];
        let p_dot = self.params.do_kappa2 * sig_pow_vec(mu_dot, self.params.do_n1, self.params.smooth_eps)
            + self.params.do_kappa3 * sig_pow_vec(mu_dot, self.params.do_n2, self.params.smooth_eps);
        self.state.p_state[i] += p_dot * dt;

        let theta_dot = self.params.do_kappa1 * p_dot
            + self.params.do_eta1 * sig_pow_vec(s, self.params.do_n1, self.params.smooth_eps)
            + self.params.do_eta2 * sig_pow_vec(s, self.params.do_n2, self.params.smooth_eps)
            + self.params.do_eta3 * sign_vec(s, self.params.smooth_eps);
        self.state.theta_hat[i] += theta_dot * dt;

        self.state.mu_prev[i] = mu;
        self.state.mu_dot_prev[i] = mu_dot;
    }

    fn update_attitude(&mut self, i: usize, u: Vector3<f64>, mass: f64) {
        let psi = self.params.desired_yaw;
        let sin_psi = psi.sin();
        let cos_psi = psi.cos();
        let uz = u.z + self.params.gravity;
        let t = (u.x * u.x + u.y * u.y + uz * uz).sqrt().max(1.0e-6);
        let phi_arg = (u.x * sin_psi - u.y * cos_psi) / t;
        let phi = phi_arg.clamp(-1.0, 1.0).asin();
        let theta = (u.x * cos_psi + u.y * sin_psi).atan2(uz);
        self.state.attitudes[i] = Vector3::new(phi, theta, psi);
        self.state.thrusts[i] = t * mass;
    }

    fn leader_state(&mut self, t: f64) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        if self.params.leader_paused {
            if let Some(state) = self.state.leader_hold {
                return state;
            }
            let scaled = t * self.params.leader_time_scale;
            let state = self.params.leader.state(scaled);
            self.state.leader_hold = Some(state);
            return state;
        }
        self.state.leader_hold = None;
        let scaled = t * self.params.leader_time_scale;
        self.params.leader.state(scaled)
    }

    fn build_constraints(
        &self,
        p: Vector3<f64>,
        v: Vector3<f64>,
        gamma: f64,
        theta_hat: &Vector3<f64>,
        t: f64,
    ) -> Vec<Halfspace3> {
        let pi1 = self.params.lambda1 * self.params.lambda2;
        let pi2 = self.params.lambda1 + self.params.lambda2;
        let mut out = Vec::new();

        for ob in &self.params.obstacles {
            let p_ob = ob.pos(t);
            let r = p - p_ob;
            let r2 = r.norm_squared();
            if r2 < 1.0e-10 {
                continue;
            }
            let d2 = ob.d * ob.d;
            let h = r2 - d2;

            let (_v_rel, c_base, lfh) = if self.params.use_moving_obstacle_terms {
                let v_ob = ob.vel(t);
                let a_ob = ob.acc();
                let v_rel = v - v_ob;
                let lfh = 2.0 * r.dot(&v_rel);
                let c = 2.0 * v_rel.dot(&v_rel) + 2.0 * r.dot(&(gamma * v - a_ob));
                (v_rel, c, lfh)
            } else {
                let lfh = 2.0 * r.dot(&v);
                let c = 2.0 * v.dot(&v) + 2.0 * r.dot(&(gamma * v));
                (v, c, lfh)
            };

            let delta1 = 2.0 * r.norm() * (theta_hat.norm() + self.params.delta_theta);
            let phi = pi1 * h + pi2 * lfh - delta1;

            let xi1 = 1.0 + self.params.delta2_star;
            let xi2 = 1.0 - self.params.delta2_star;

            let a1 = r * (2.0 * xi1);
            let b1 = -phi - xi1 * c_base;
            out.push(Halfspace3 { a: a1, b: b1 });

            if self.params.delta2_star > 0.0 {
                let a2 = r * (2.0 * xi2);
                let b2 = -phi - xi2 * c_base;
                out.push(Halfspace3 { a: a2, b: b2 });
            }
        }

        out
    }

    fn formation_offset(&self, i: usize) -> Vector3<f64> {
        if i < self.state.formation_offsets.len() {
            self.state.formation_offsets[i]
        } else {
            Vector3::new(0.0, 0.0, 0.0)
        }
    }

    fn adj_weight(&self, i: usize, j: usize, n: usize) -> f64 {
        if self.params.adjacency.len() == n && self.params.adjacency[i].len() == n {
            self.params.adjacency[i][j]
        } else {
            if i == j { 0.0 } else { 1.0 }
        }
    }

    fn leader_link(&self, i: usize, n: usize) -> f64 {
        if self.params.leader_links.len() == n {
            self.params.leader_links[i]
        } else {
            1.0
        }
    }

    fn a1(&self) -> f64 {
        2.0 - self.params.m1 / self.params.m2.max(1.0e-6)
    }

    fn a2(&self) -> f64 {
        self.params.m1 / self.params.m2.max(1.0e-6)
    }

    pub fn attitudes_flat(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.state.attitudes.len() * 3);
        for a in &self.state.attitudes {
            out.push(a.x as f32);
            out.push(a.y as f32);
            out.push(a.z as f32);
        }
        out
    }

    pub fn reset_agent(&mut self, index: usize, _pos: Vector3<f64>, vel: Vector3<f64>) {
        if index >= self.state.chi.len() {
            return;
        }
        self.state.v_hat[index] = vel;
        self.state.theta_hat[index] = Vector3::new(0.0, 0.0, 0.0);
        self.state.p_state[index] = Vector3::new(0.0, 0.0, 0.0);
        self.state.mu_prev[index] = Vector3::new(0.0, 0.0, 0.0);
        self.state.mu_dot_prev[index] = Vector3::new(0.0, 0.0, 0.0);
        self.state.alpha_prev[index] = vel;
        self.state.alpha_dot_prev[index] = Vector3::new(0.0, 0.0, 0.0);
        self.state.mu_ready[index] = false;
        self.state.alpha_ready[index] = false;
    }
}

fn vec3_from(v: [f64; 3]) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
}

fn smooth_sign(x: f64, eps: f64) -> f64 {
    x / (x * x + eps * eps).sqrt()
}

fn sig_pow(x: f64, a: f64, eps: f64) -> f64 {
    let s = (x * x + eps * eps).sqrt();
    if s <= 0.0 {
        0.0
    } else {
        s.powf(a - 1.0) * x
    }
}

fn sign_vec(v: Vector3<f64>, eps: f64) -> Vector3<f64> {
    Vector3::new(smooth_sign(v.x, eps), smooth_sign(v.y, eps), smooth_sign(v.z, eps))
}

fn sig_pow_vec(v: Vector3<f64>, a: f64, eps: f64) -> Vector3<f64> {
    Vector3::new(sig_pow(v.x, a, eps), sig_pow(v.y, a, eps), sig_pow(v.z, a, eps))
}
