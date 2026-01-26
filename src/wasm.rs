#![cfg(target_arch = "wasm32")]

use crate::algorithms::flocking::FlockParams;
use crate::algorithms::flocking_alpha::FlockAlphaParams;
use crate::algorithms::formation_ecbf::FormationEcbfParams;
use crate::algorithms::safe_flocking_alpha::SafeFlockAlphaParams;
use crate::engine::{
    algorithm_catalog, model_catalog, AlgorithmInfo, Engine, ModelInfo, ALGO_FLOCKING, MODEL_RING,
};
use nalgebra::Vector3;
use serde::Deserialize;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn available_models() -> js_sys::Array {
    let out = js_sys::Array::new();
    for info in model_catalog() {
        out.push(&model_info_to_js(info));
    }
    out
}

#[wasm_bindgen]
pub fn available_algorithms() -> js_sys::Array {
    let out = js_sys::Array::new();
    for info in algorithm_catalog() {
        out.push(&algorithm_info_to_js(info));
    }
    out
}

#[wasm_bindgen]
pub fn algorithms_for_model(model_id: &str) -> js_sys::Array {
    let out = js_sys::Array::new();
    for info in algorithm_catalog().iter().filter(|a| a.compatible_models.contains(&model_id)) {
        out.push(&algorithm_info_to_js(info));
    }
    out
}

#[wasm_bindgen]
pub fn flocking_defaults() -> JsValue {
    let params = FlockParams::default();
    serde_wasm_bindgen::to_value(&params).unwrap_or(JsValue::NULL)
}

#[wasm_bindgen]
pub fn flocking_alpha_defaults() -> JsValue {
    let params = FlockAlphaParams::default();
    serde_wasm_bindgen::to_value(&params).unwrap_or(JsValue::NULL)
}

#[wasm_bindgen]
pub fn formation_ecbf_defaults() -> JsValue {
    let params = FormationEcbfParams::default();
    serde_wasm_bindgen::to_value(&params).unwrap_or(JsValue::NULL)
}

#[wasm_bindgen]
pub fn safe_flocking_alpha_defaults() -> JsValue {
    let params = SafeFlockAlphaParams::default();
    serde_wasm_bindgen::to_value(&params).unwrap_or(JsValue::NULL)
}

fn model_info_to_js(info: &ModelInfo) -> JsValue {
    let obj = js_sys::Object::new();
    let _ = js_sys::Reflect::set(&obj, &JsValue::from_str("id"), &JsValue::from_str(info.id));
    let _ = js_sys::Reflect::set(&obj, &JsValue::from_str("name"), &JsValue::from_str(info.name));
    let _ = js_sys::Reflect::set(
        &obj,
        &JsValue::from_str("description"),
        &JsValue::from_str(info.description),
    );
    let _ = js_sys::Reflect::set(
        &obj,
        &JsValue::from_str("defaultAlgorithm"),
        &JsValue::from_str(info.default_algorithm),
    );
    JsValue::from(obj)
}

fn algorithm_info_to_js(info: &AlgorithmInfo) -> JsValue {
    let obj = js_sys::Object::new();
    let _ = js_sys::Reflect::set(&obj, &JsValue::from_str("id"), &JsValue::from_str(info.id));
    let _ = js_sys::Reflect::set(&obj, &JsValue::from_str("name"), &JsValue::from_str(info.name));
    let _ = js_sys::Reflect::set(
        &obj,
        &JsValue::from_str("description"),
        &JsValue::from_str(info.description),
    );
    let compatible = js_sys::Array::new();
    for m in info.compatible_models {
        compatible.push(&JsValue::from_str(m));
    }
    let _ = js_sys::Reflect::set(&obj, &JsValue::from_str("compatible"), &compatible);
    JsValue::from(obj)
}

#[wasm_bindgen]
pub struct WasmSim {
    engine: Engine,
}

#[wasm_bindgen]
impl WasmSim {
    #[wasm_bindgen(constructor)]
    pub fn new(states: Vec<f64>, dt: f64) -> Result<WasmSim, JsValue> {
        let engine = Engine::new_from_states(states, dt, Some(ALGO_FLOCKING))
            .map_err(|e| JsValue::from_str(&e))?;
        Ok(WasmSim { engine })
    }

    #[wasm_bindgen(js_name = "newWithIds")]
    pub fn new_with_ids(model_id: &str, algorithm_id: &str) -> Result<WasmSim, JsValue> {
        let engine = Engine::new_builtin(model_id, Some(algorithm_id))
            .map_err(|e| JsValue::from_str(&e))?;
        Ok(WasmSim { engine })
    }

    pub fn new_demo() -> WasmSim {
        let engine = Engine::new_builtin(MODEL_RING, Some(ALGO_FLOCKING))
            .expect("ring-swarm model must exist");
        WasmSim { engine }
    }

    /// Build simulation from a custom config object:
    /// {
    ///   dt?: number,
    ///   algorithm?: string,
    ///   plane2d?: bool,
    ///   agents?: [{ position: [f64;3], velocity: [f64;3], mass?: f64, drag?: f64, group?: usize }],
    ///   clusters?: [{ count, center:[x,y,z], radius, velocity?:[vx,vy,vz], radialSpeed?: f64, drag?: f64, group?: usize }]
    /// }
    #[wasm_bindgen(js_name = "newFromConfig")]
    pub fn new_from_config(config: JsValue) -> Result<WasmSim, JsValue> {
        let cfg: CustomConfig = serde_wasm_bindgen::from_value(config)
            .map_err(|e| JsValue::from_str(&format!("invalid config: {}", e)))?;
        let (configs, dt, plane_2d, algo) = build_custom_configs(cfg)?;
        let engine = Engine::new_custom(configs, dt, algo.as_deref(), plane_2d)
            .map_err(|e| JsValue::from_str(&e))?;
        Ok(WasmSim { engine })
    }

    pub fn len(&self) -> usize { self.engine.len() }

    pub fn step(&mut self) { self.engine.step(); }

    pub fn tick(&mut self) { self.engine.tick(); }

    pub fn set_plane_2d(&mut self, enabled: bool) { self.engine.set_plane_2d(enabled); }

    pub fn set_force(&mut self, index: usize, fx: f64, fy: f64, fz: f64) {
        self.engine.set_force(index, Vector3::new(fx, fy, fz));
    }

    pub fn set_position(&mut self, index: usize, x: f64, y: f64, z: f64) {
        self.engine.set_position(index, Vector3::new(x, y, z));
    }

    pub fn set_velocity(&mut self, index: usize, x: f64, y: f64, z: f64) {
        self.engine.set_velocity(index, Vector3::new(x, y, z));
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
        let pos = Vector3::new(px, py, pz);
        let vel = Vector3::new(vx, vy, vz);
        self.engine.set_position(index, pos);
        self.engine.set_velocity(index, vel);
        self.engine.reset_agent(index, pos, vel);
    }

    pub fn set_uniform_force(&mut self, fx: f64, fy: f64, fz: f64) {
        self.engine.set_uniform_force(Vector3::new(fx, fy, fz));
    }

    pub fn set_flock_params(&mut self, params: JsValue) -> Result<(), JsValue> {
        let params: FlockParams = serde_wasm_bindgen::from_value(params)
            .map_err(|e| JsValue::from_str(&format!("invalid flock params: {}", e)))?;
        self.engine
            .set_flock_params(params)
            .map_err(|e| JsValue::from_str(&e))
    }

    pub fn set_flock_alpha_params(&mut self, params: JsValue) -> Result<(), JsValue> {
        let params: FlockAlphaParams = serde_wasm_bindgen::from_value(params)
            .map_err(|e| JsValue::from_str(&format!("invalid flock-alpha params: {}", e)))?;
        self.engine
            .set_flock_alpha_params(params)
            .map_err(|e| JsValue::from_str(&e))
    }

    pub fn set_formation_ecbf_params(&mut self, params: JsValue) -> Result<(), JsValue> {
        let params: FormationEcbfParams = serde_wasm_bindgen::from_value(params)
            .map_err(|e| JsValue::from_str(&format!("invalid formation-ecbf params: {}", e)))?;
        self.engine
            .set_formation_ecbf_params(params)
            .map_err(|e| JsValue::from_str(&e))
    }

    pub fn set_safe_flocking_alpha_params(&mut self, params: JsValue) -> Result<(), JsValue> {
        let params: SafeFlockAlphaParams = serde_wasm_bindgen::from_value(params)
            .map_err(|e| JsValue::from_str(&format!("invalid safe-flocking-alpha params: {}", e)))?;
        self.engine
            .set_safe_flocking_alpha_params(params)
            .map_err(|e| JsValue::from_str(&e))
    }

    pub fn positions(&self) -> Vec<f32> { self.engine.positions_flat() }

    pub fn states(&self) -> Vec<f32> { self.engine.state_matrix_flat() }

    pub fn debug_states(&self) -> Vec<f32> { self.engine.debug_states_flat() }

    pub fn dt(&self) -> f64 { self.engine.dt() }

    pub fn set_algorithm(&mut self, algorithm_id: &str) -> Result<(), JsValue> {
        self.engine
            .set_algorithm(algorithm_id)
            .map_err(|e| JsValue::from_str(&e))
    }

    pub fn groups(&self) -> Vec<u32> { self.engine.groups() }

    pub fn attitudes(&self) -> Vec<f32> { self.engine.attitudes_flat() }
}

#[derive(Debug, Deserialize)]
struct CustomConfig {
    #[serde(default)]
    dt: Option<f64>,
    #[serde(default)]
    algorithm: Option<String>,
    #[serde(default)]
    plane2d: Option<bool>,
    #[serde(default)]
    agents: Option<Vec<CustomAgent>>,
    #[serde(default)]
    clusters: Option<Vec<CustomCluster>>,
}

#[derive(Debug, Deserialize)]
struct CustomAgent {
    position: [f64; 3],
    velocity: [f64; 3],
    #[serde(default)]
    mass: Option<f64>,
    #[serde(default)]
    drag: Option<f64>,
    #[serde(default)]
    group: Option<usize>,
}

#[derive(Debug, Deserialize)]
struct CustomCluster {
    #[serde(default = "default_shape")]
    shape: String,
    count: usize,
    center: [f64; 3],
    radius: f64,
    #[serde(default)]
    velocity: Option<[f64; 3]>,
    #[serde(default, rename = "radialSpeed")]
    radial_speed: Option<f64>,
    #[serde(default)]
    drag: Option<f64>,
    #[serde(default)]
    group: Option<usize>,
    #[serde(default)]
    mass: Option<f64>,
}

fn default_shape() -> String { "sphere".to_string() }

fn build_custom_configs(cfg: CustomConfig) -> Result<(Vec<crate::BodyConfig>, f64, bool, Option<String>), JsValue> {
    let dt = cfg.dt.unwrap_or(crate::models::particles::DEMO_DT);
    let plane_2d = cfg.plane2d.unwrap_or(false);
    let mut out = Vec::new();

    if let Some(agents) = cfg.agents {
        for a in agents {
            out.push(crate::BodyConfig {
                mass: a.mass.unwrap_or(1.0),
                state: [
                    a.position[0],
                    a.position[1],
                    a.position[2],
                    a.velocity[0],
                    a.velocity[1],
                    a.velocity[2],
                ],
                drag_coefficient: a.drag.unwrap_or(0.0),
                trajectory_write: false,
                group: a.group.unwrap_or(0),
            });
        }
    }

    if let Some(clusters) = cfg.clusters {
        for cluster in clusters {
            let generated = build_cluster(&cluster)?;
            out.extend(generated);
        }
    }

    Ok((out, dt, plane_2d, cfg.algorithm))
}

fn build_cluster(c: &CustomCluster) -> Result<Vec<crate::BodyConfig>, JsValue> {
    match c.shape.as_str() {
        "sphere" | "ball" => build_sphere_cluster(c),
        "circle" | "ring" => build_circle_cluster(c),
        other => Err(JsValue::from_str(&format!("unknown cluster shape '{}'", other))),
    }
}

fn build_sphere_cluster(c: &CustomCluster) -> Result<Vec<crate::BodyConfig>, JsValue> {
    let count = c.count;
    if count == 0 {
        return Ok(Vec::new());
    }
    let mut configs = Vec::with_capacity(count);
    // Fibonacci sphere distribution for deterministic spread.
    let golden = (1.0 + 5.0_f64.sqrt()) * 0.5;
    let ga = 2.0 - 1.0 / golden;
    for i in 0..count {
        let fi = i as f64 + 0.5;
        let z = 1.0 - (2.0 * fi) / count as f64;
        let r = (1.0 - z * z).max(0.0).sqrt();
        let theta = 2.0 * std::f64::consts::PI * fi * ga;
        let x = theta.cos() * r;
        let y = theta.sin() * r;
        let pos = Vector3::new(x, y, z) * c.radius + Vector3::new(c.center[0], c.center[1], c.center[2]);

        let base_vel = c.velocity.unwrap_or([0.0, 0.0, 0.0]);
        let mut vel = Vector3::new(base_vel[0], base_vel[1], base_vel[2]);
        if let Some(radial) = c.radial_speed {
            let dir = Vector3::new(x, y, z).normalize();
            vel += dir * radial;
        }

        configs.push(crate::BodyConfig {
            mass: c.mass.unwrap_or(1.0),
            state: [pos.x, pos.y, pos.z, vel.x, vel.y, vel.z],
            drag_coefficient: c.drag.unwrap_or(0.0),
            trajectory_write: false,
            group: c.group.unwrap_or(0),
        });
    }
    Ok(configs)
}

fn build_circle_cluster(c: &CustomCluster) -> Result<Vec<crate::BodyConfig>, JsValue> {
    let count = c.count.max(1);
    let mut configs = Vec::with_capacity(count);
    for i in 0..count {
        let angle = (i as f64 / count as f64) * 2.0 * std::f64::consts::PI;
        let pos = Vector3::new(
            c.center[0] + c.radius * angle.cos(),
            c.center[1] + c.radius * angle.sin(),
            c.center[2],
        );

        let base_vel = c.velocity.unwrap_or([0.0, 0.0, 0.0]);
        let mut vel = Vector3::new(base_vel[0], base_vel[1], base_vel[2]);
        if let Some(radial) = c.radial_speed {
            let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
            vel += dir * radial;
        }

        configs.push(crate::BodyConfig {
            mass: c.mass.unwrap_or(1.0),
            state: [pos.x, pos.y, pos.z, vel.x, vel.y, vel.z],
            drag_coefficient: c.drag.unwrap_or(0.0),
            trajectory_write: false,
            group: c.group.unwrap_or(0),
        });
    }
    Ok(configs)
}
