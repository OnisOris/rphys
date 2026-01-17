#![cfg(target_arch = "wasm32")]

use nalgebra::Vector3;
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;

use crate::{BodyConfig, Simulator};

const DEMO_COUNT: usize = 2000;

const DEMO_DT: f64 = 1.0 / 60.0;
const DEMO_RADIUS: f64 = 14.5;
const DEMO_Z_STEP: f64 = 0.35;
const DEMO_Z_OFFSET: f64 = 1.225;
const DEMO_DRAG: f64 = 0.08;

const FLOCK_NEIGHBOR_RADIUS: f64 = 2.6;
const FLOCK_SEPARATION_RADIUS: f64 = 0.9;
const FLOCK_COHESION_WEIGHT: f64 = 0.45;
const FLOCK_ALIGNMENT_WEIGHT: f64 = 0.65;
const FLOCK_SEPARATION_WEIGHT: f64 = 10.35;
const FLOCK_BOUNDARY_RADIUS: f64 = 6.0;
const FLOCK_BOUNDARY_WEIGHT: f64 = 0.8;
const FLOCK_MAX_SPEED: f64 = 2.4;
const FLOCK_MAX_FORCE: f64 = 1.6;
const FLOCK_SPEED_LIMIT: f64 = 2.;

struct FlockParams {
    neighbor_radius: f64,
    separation_radius: f64,
    cohesion_weight: f64,
    alignment_weight: f64,
    separation_weight: f64,
    boundary_radius: f64,
    boundary_weight: f64,
    max_speed: f64,
    max_force: f64,
    speed_limit: f64,
}

impl Default for FlockParams {
    fn default() -> Self {
        Self {
            neighbor_radius: FLOCK_NEIGHBOR_RADIUS,
            separation_radius: FLOCK_SEPARATION_RADIUS,
            cohesion_weight: FLOCK_COHESION_WEIGHT,
            alignment_weight: FLOCK_ALIGNMENT_WEIGHT,
            separation_weight: FLOCK_SEPARATION_WEIGHT,
            boundary_radius: FLOCK_BOUNDARY_RADIUS,
            boundary_weight: FLOCK_BOUNDARY_WEIGHT,
            max_speed: FLOCK_MAX_SPEED,
            max_force: FLOCK_MAX_FORCE,
            speed_limit: FLOCK_SPEED_LIMIT,
        }
    }
}

fn configs_from_states(states: &[f64]) -> Result<Vec<BodyConfig>, JsValue> {
    if states.len() % 6 != 0 {
        return Err(JsValue::from_str("states length must be a multiple of 6"));
    }

    let n = states.len() / 6;
    let mut configs = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * 6;
        configs.push(BodyConfig {
            mass: 1.0,
            state: [
                states[base],
                states[base + 1],
                states[base + 2],
                states[base + 3],
                states[base + 4],
                states[base + 5],
            ],
            drag_coefficient: 0.0,
            trajectory_write: false,
            group: 0,
        });
    }

    Ok(configs)
}

fn demo_configs(count: usize) -> Vec<BodyConfig> {
    let mut configs = Vec::with_capacity(count);
    if count == 0 {
        return configs;
    }

    let count_f = count as f64;
    for i in 0..count {
        let i_f = i as f64;
        let angle = (i_f / count_f) * 2.0 * PI;
        let wobble = 0.6 + 0.4 * (i_f * 0.7).sin();
        let r = DEMO_RADIUS * wobble;
        let x = r * angle.cos();
        let y = r * angle.sin();
        let z = (i % 8) as f64 * DEMO_Z_STEP - DEMO_Z_OFFSET;
        let tangent = Vector3::new(-y, x, 0.0);
        let tangent = if tangent.norm_squared() > 1.0e-12 {
            tangent.normalize()
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        };
        let speed = 1.0 + 0.4 * (i_f * 1.3).sin();
        let vz = 0.3 * (i_f * 0.9).cos();
        let vel = tangent * speed + Vector3::new(0.0, 0.0, vz);

        configs.push(BodyConfig {
            mass: 1.0,
            state: [x, y, z, vel.x, vel.y, vel.z],
            drag_coefficient: DEMO_DRAG,
            trajectory_write: false,
            group: 0,
        });
    }

    configs
}

#[wasm_bindgen]
pub struct WasmSim {
    sim: Simulator,
    flock: FlockParams,
    plane_2d: bool,
}

impl WasmSim {
    fn flatten_to_plane(&mut self) {
        let positions = self.sim.positions().to_vec();
        let velocities = self.sim.velocities().to_vec();
        for (i, (mut p, mut v)) in positions
            .into_iter()
            .zip(velocities.into_iter())
            .enumerate()
        {
            p.z = 0.0;
            v.z = 0.0;
            self.sim.set_position(i, p);
            self.sim.set_velocity(i, v);
        }
    }

    fn apply_flock_forces(&mut self) {
        let mut positions = self.sim.positions().to_vec();
        let mut velocities = self.sim.velocities().to_vec();
        if self.plane_2d {
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

        let neighbor_r2 = self.flock.neighbor_radius * self.flock.neighbor_radius;
        let separation_r2 = self.flock.separation_radius * self.flock.separation_radius;
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
                force += (avg_pos - pos_i) * self.flock.cohesion_weight;
                force += (avg_vel - vel_i) * self.flock.alignment_weight;
            }

            if close > 0 {
                let inv = 1.0 / close as f64;
                force += separation_sum * inv * self.flock.separation_weight;
            }

            let dist = pos_i.norm();
            if dist > self.flock.boundary_radius && dist > 0.0 {
                let dir = pos_i / dist;
                force += -dir * (dist - self.flock.boundary_radius) * self.flock.boundary_weight;
            }

            let speed = vel_i.norm();
            if speed > self.flock.max_speed && speed > 0.0 {
                let dir = vel_i / speed;
                force += -dir * (speed - self.flock.max_speed) * self.flock.speed_limit;
            }

            let fmag = force.norm();
            if fmag > self.flock.max_force && fmag > 0.0 {
                force = force / fmag * self.flock.max_force;
            }

            if self.plane_2d {
                force.z = 0.0;
            }
            forces.push(force);
        }

        for (i, f) in forces.into_iter().enumerate() {
            self.sim.set_force(i, f);
        }
    }
}

#[wasm_bindgen]
impl WasmSim {
    #[wasm_bindgen(constructor)]
    pub fn new(states: Vec<f64>, dt: f64) -> Result<WasmSim, JsValue> {
        let configs = configs_from_states(&states)?;

        Ok(WasmSim {
            sim: Simulator::new(&configs, dt),
            flock: FlockParams::default(),
            plane_2d: false,
        })
    }

    pub fn new_demo() -> WasmSim {
        let configs = demo_configs(DEMO_COUNT);
        WasmSim {
            sim: Simulator::new(&configs, DEMO_DT),
            flock: FlockParams::default(),
            plane_2d: false,
        }
    }

    pub fn len(&self) -> usize {
        self.sim.len()
    }

    pub fn step(&mut self) {
        self.sim.step();
    }

    pub fn tick(&mut self) {
        if self.plane_2d {
            self.flatten_to_plane();
        }
        self.apply_flock_forces();
        self.sim.step();
        if self.plane_2d {
            self.flatten_to_plane();
        }
    }

    pub fn set_plane_2d(&mut self, enabled: bool) {
        self.plane_2d = enabled;
        if self.plane_2d {
            self.flatten_to_plane();
        }
    }

    pub fn set_force(&mut self, index: usize, fx: f64, fy: f64, fz: f64) {
        self.sim.set_force(index, Vector3::new(fx, fy, fz));
    }

    pub fn set_position(&mut self, index: usize, x: f64, y: f64, z: f64) {
        self.sim.set_position(index, Vector3::new(x, y, z));
    }

    pub fn set_velocity(&mut self, index: usize, x: f64, y: f64, z: f64) {
        self.sim.set_velocity(index, Vector3::new(x, y, z));
    }

    pub fn set_position_and_velocity(
        &mut self,
        index: usize,
        px: f64,
        py: f64,
        pz: f64,
        vx: f64,
        vy: f64,
        vz: f64,
    ) {
        self.sim.set_position(index, Vector3::new(px, py, pz));
        self.sim.set_velocity(index, Vector3::new(vx, vy, vz));
    }

    pub fn set_uniform_force(&mut self, fx: f64, fy: f64, fz: f64) {
        let f = Vector3::new(fx, fy, fz);
        for i in 0..self.sim.len() {
            self.sim.set_force(i, f);
        }
    }

    pub fn positions(&self) -> Vec<f32> {
        let positions = self.sim.positions();
        let mut out = Vec::with_capacity(positions.len() * 3);
        for p in positions {
            out.push(p.x as f32);
            out.push(p.y as f32);
            out.push(p.z as f32);
        }
        out
    }
}
