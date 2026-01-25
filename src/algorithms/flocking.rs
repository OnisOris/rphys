use crate::models::particles::ParticleModel;
use nalgebra::Vector3;

pub const DEFAULT_NEIGHBOR_RADIUS: f64 = 2.6;
pub const DEFAULT_SEPARATION_RADIUS: f64 = 0.9;
pub const DEFAULT_COHESION_WEIGHT: f64 = 0.45;
pub const DEFAULT_ALIGNMENT_WEIGHT: f64 = 0.65;
pub const DEFAULT_SEPARATION_WEIGHT: f64 = 10.35;
pub const DEFAULT_BOUNDARY_RADIUS: f64 = 6.0;
pub const DEFAULT_BOUNDARY_WEIGHT: f64 = 0.8;
pub const DEFAULT_MAX_SPEED: f64 = 2.4;
pub const DEFAULT_MAX_FORCE: f64 = 1.6;
pub const DEFAULT_SPEED_LIMIT: f64 = 2.0;

#[derive(Debug, Clone)]
pub struct FlockParams {
    pub neighbor_radius: f64,
    pub separation_radius: f64,
    pub cohesion_weight: f64,
    pub alignment_weight: f64,
    pub separation_weight: f64,
    pub boundary_radius: f64,
    pub boundary_weight: f64,
    pub max_speed: f64,
    pub max_force: f64,
    pub speed_limit: f64,
}

impl Default for FlockParams {
    fn default() -> Self {
        Self {
            neighbor_radius: DEFAULT_NEIGHBOR_RADIUS,
            separation_radius: DEFAULT_SEPARATION_RADIUS,
            cohesion_weight: DEFAULT_COHESION_WEIGHT,
            alignment_weight: DEFAULT_ALIGNMENT_WEIGHT,
            separation_weight: DEFAULT_SEPARATION_WEIGHT,
            boundary_radius: DEFAULT_BOUNDARY_RADIUS,
            boundary_weight: DEFAULT_BOUNDARY_WEIGHT,
            max_speed: DEFAULT_MAX_SPEED,
            max_force: DEFAULT_MAX_FORCE,
            speed_limit: DEFAULT_SPEED_LIMIT,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Flocking {
    pub params: FlockParams,
}

impl Flocking {
    pub fn new(params: FlockParams) -> Self {
        Self { params }
    }

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

        let neighbor_r2 = self.params.neighbor_radius * self.params.neighbor_radius;
        let separation_r2 = self.params.separation_radius * self.params.separation_radius;
        let mut forces = Vec::with_capacity(n);

        for i in 0..n {
            let pos_i = positions[i];
            let vel_i = velocities[i];
            let mut cohesion_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut alignment_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut separation_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut neighbors = 0usize;
            let mut close = 0usize;

            for j in 0..n {
                if i == j {
                    continue;
                }
                let diff = pos_i - positions[j];
                let dist2 = diff.norm_squared();
                if dist2 < neighbor_r2 {
                    cohesion_sum += positions[j];
                    alignment_sum += velocities[j];
                    neighbors += 1;
                }
                if dist2 < separation_r2 && dist2 > 1.0e-12 {
                    let dist = dist2.sqrt();
                    separation_sum += diff / dist;
                    close += 1;
                }
            }

            let mut force = Vector3::new(0.0, 0.0, 0.0);

            if neighbors > 0 {
                let inv = 1.0 / neighbors as f64;
                let avg_pos = cohesion_sum * inv;
                let avg_vel = alignment_sum * inv;
                force += (avg_pos - pos_i) * self.params.cohesion_weight;
                force += (avg_vel - vel_i) * self.params.alignment_weight;
            }

            if close > 0 {
                let inv = 1.0 / close as f64;
                force += separation_sum * inv * self.params.separation_weight;
            }

            let dist = pos_i.norm();
            if dist > self.params.boundary_radius && dist > 0.0 {
                let dir = pos_i / dist;
                force += -dir * (dist - self.params.boundary_radius) * self.params.boundary_weight;
            }

            let speed = vel_i.norm();
            if speed > self.params.max_speed && speed > 0.0 {
                let dir = vel_i / speed;
                force += -dir * (speed - self.params.max_speed) * self.params.speed_limit;
            }

            let fmag = force.norm();
            if fmag > self.params.max_force && fmag > 0.0 {
                force = force / fmag * self.params.max_force;
            }

            if plane_2d {
                forces.push(Vector3::new(force.x, force.y, 0.0));
            } else {
                forces.push(force);
            }
        }

        for (i, f) in forces.into_iter().enumerate() {
            model.set_force(i, f);
        }
    }
}
