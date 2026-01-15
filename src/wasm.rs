#![cfg(target_arch = "wasm32")]

use nalgebra::Vector3;
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;

use crate::{BodyConfig, Simulator};

const DEMO_COUNT: usize = 64;
const DEMO_DT: f64 = 1.0 / 60.0;
const DEMO_RADIUS: f64 = 4.0;
const DEMO_SPRING_K: f64 = 0.6;
const DEMO_Z_STEP: f64 = 0.3;
const DEMO_Z_OFFSET: f64 = 1.05;

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

        configs.push(BodyConfig {
            mass: 1.0,
            state: [x, y, z, 0.0, 0.0, 0.0],
            drag_coefficient: 0.0,
            trajectory_write: false,
            group: 0,
        });
    }

    configs
}

#[wasm_bindgen]
pub struct WasmSim {
    sim: Simulator,
    spring_k: f64,
}

impl WasmSim {
    fn apply_spring_forces(&mut self) {
        let forces: Vec<Vector3<f64>> = {
            let positions = self.sim.positions();
            positions
                .iter()
                .map(|p| Vector3::new(-self.spring_k * p.x, -self.spring_k * p.y, -self.spring_k * p.z))
                .collect()
        };

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
            spring_k: 0.0,
        })
    }

    pub fn new_demo() -> WasmSim {
        let configs = demo_configs(DEMO_COUNT);
        WasmSim {
            sim: Simulator::new(&configs, DEMO_DT),
            spring_k: DEMO_SPRING_K,
        }
    }

    pub fn len(&self) -> usize {
        self.sim.len()
    }

    pub fn step(&mut self) {
        self.sim.step();
    }

    pub fn tick(&mut self) {
        self.apply_spring_forces();
        self.sim.step();
    }

    pub fn set_spring_k(&mut self, k: f64) {
        self.spring_k = k;
    }

    pub fn set_force(&mut self, index: usize, fx: f64, fy: f64, fz: f64) {
        self.sim.set_force(index, Vector3::new(fx, fy, fz));
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
