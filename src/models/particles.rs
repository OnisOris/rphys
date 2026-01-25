use crate::{BodyConfig, Simulator};
use nalgebra::Vector3;
use std::f64::consts::PI;

pub const DEMO_COUNT: usize = 20;
pub const DEMO_DT: f64 = 1.0 / 60.0;
pub const DEMO_DRAG: f64 = 0.08;

/// Simple N-body particle model backed by the shared Simulator.
#[derive(Debug)]
pub struct ParticleModel {
    sim: Simulator,
}

impl ParticleModel {
    pub fn new(configs: Vec<BodyConfig>, dt: f64) -> Self {
        Self {
            sim: Simulator::new(&configs, dt),
        }
    }

    pub fn from_states(states: &[f64], dt: f64, drag_coefficient: f64) -> Result<Self, String> {
        let configs = configs_from_states(states, drag_coefficient)?;
        Ok(Self::new(configs, dt))
    }

    pub fn len(&self) -> usize { self.sim.len() }

    pub fn dt(&self) -> f64 { self.sim.dt() }

    pub fn set_dt(&mut self, dt: f64) { self.sim.set_dt(dt); }

    pub fn positions(&self) -> &[Vector3<f64>] { self.sim.positions() }

    pub fn velocities(&self) -> &[Vector3<f64>] { self.sim.velocities() }

    pub fn set_position(&mut self, i: usize, pos: Vector3<f64>) { self.sim.set_position(i, pos); }

    pub fn set_velocity(&mut self, i: usize, vel: Vector3<f64>) { self.sim.set_velocity(i, vel); }

    pub fn groups(&self) -> &[usize] { self.sim.groups() }

    pub fn set_force(&mut self, i: usize, f: Vector3<f64>) { self.sim.set_force(i, f); }

    pub fn set_uniform_force(&mut self, f: Vector3<f64>) {
        for i in 0..self.sim.len() {
            self.sim.set_force(i, f);
        }
    }

    pub fn step(&mut self) { self.sim.step(); }

    pub fn state_matrix(&self) -> Vec<[f64; 6]> { self.sim.state_matrix() }

    /// Force positions/velocities into the XY plane.
    pub fn flatten_to_plane(&mut self) {
        let positions = self.sim.positions().to_vec();
        let velocities = self.sim.velocities().to_vec();
        for (i, (mut p, mut v)) in positions.into_iter().zip(velocities.into_iter()).enumerate() {
            p.z = 0.0;
            v.z = 0.0;
            self.sim.set_position(i, p);
            self.sim.set_velocity(i, v);
        }
    }
}

/// Convert packed states [x,y,z,vx,vy,vz]* into BodyConfig list.
pub fn configs_from_states(states: &[f64], drag_coefficient: f64) -> Result<Vec<BodyConfig>, String> {
    if states.len() % 6 != 0 {
        return Err("states length must be a multiple of 6".to_string());
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
            drag_coefficient,
            trajectory_write: false,
            group: 0,
        });
    }
    Ok(configs)
}

/// Ring-with-wobble demo used by the default WASM scene.
pub fn ring_demo_configs(count: usize) -> Vec<BodyConfig> {
    let mut configs = Vec::with_capacity(count);
    if count == 0 {
        return configs;
    }

    let count_f = count as f64;
    for i in 0..count {
        let i_f = i as f64;
        let angle = (i_f / count_f) * 2.0 * PI;
        let wobble = 0.6 + 0.4 * (i_f * 0.7).sin();
        let r = 14.5 * wobble;
        let x = r * angle.cos();
        let y = r * angle.sin();
        let z = (i % 8) as f64 * 0.35 - 1.225;
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

/// Compact lattice demo to showcase multiple models in UI.
pub fn lattice_demo_configs(side: usize) -> Vec<BodyConfig> {
    let mut configs = Vec::new();
    if side == 0 {
        return configs;
    }
    let spacing = 1.4;
    let origin = (side as f64 - 1.0) * spacing * 0.5;
    for ix in 0..side {
        for iy in 0..side {
            for iz in 0..side {
                let x = ix as f64 * spacing - origin;
                let y = iy as f64 * spacing - origin;
                let z = iz as f64 * spacing - origin;
                configs.push(BodyConfig {
                    mass: 1.0,
                    state: [x, y, z, 0.0, 0.0, 0.0],
                    drag_coefficient: 0.02,
                    trajectory_write: false,
                    group: 0,
                });
            }
        }
    }
    configs
}
