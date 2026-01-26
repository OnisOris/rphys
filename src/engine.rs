use crate::algorithms::flocking::{FlockParams, Flocking};
use crate::algorithms::flocking_alpha::{FlockAlphaParams, FlockingAlpha};
use crate::algorithms::formation_ecbf::{FormationEcbf, FormationEcbfParams};
use crate::algorithms::safe_flocking_alpha::{SafeFlockAlphaParams, SafeFlockingAlpha};
use crate::models::particles::{
    lattice_demo_configs, quadrotor_demo_configs, ring_demo_configs, ParticleModel, DEMO_COUNT,
    DEMO_DT,
};
use crate::BodyConfig;
use nalgebra::Vector3;

pub const MODEL_RING: &str = "ring-swarm";
pub const MODEL_LATTICE: &str = "lattice-swarm";
pub const MODEL_QUADROTOR: &str = "quadrotor-swarm";
pub const MODEL_FROM_STATES: &str = "from-states";

pub const ALGO_NONE: &str = "none";
pub const ALGO_FLOCKING: &str = "flocking";
pub const ALGO_FLOCKING_ALPHA: &str = "flocking-alpha";
pub const ALGO_FORMATION_ECBF: &str = "formation-ecbf";
pub const ALGO_SAFE_FLOCKING_ALPHA: &str = "safe-flocking-alpha";

pub struct ModelInfo {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub default_algorithm: &'static str,
}

pub struct AlgorithmInfo {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub compatible_models: &'static [&'static str],
}

pub fn model_catalog() -> &'static [ModelInfo] {
    &[
        ModelInfo {
            id: MODEL_RING,
            name: "Ring swarm",
            description: "Particles on a noisy ring orbit with slight vertical wobble.",
            default_algorithm: ALGO_FLOCKING,
        },
        ModelInfo {
            id: MODEL_LATTICE,
            name: "Lattice cloud",
            description: "Tight 3×3×3 lattice of particles, good for density tests.",
            default_algorithm: ALGO_FLOCKING,
        },
        ModelInfo {
            id: MODEL_QUADROTOR,
            name: "Quadrotor formation",
            description: "4-agent quadrotor formation demo (paper defaults).",
            default_algorithm: ALGO_FORMATION_ECBF,
        },
        ModelInfo {
            id: MODEL_FROM_STATES,
            name: "Custom/Clusters",
            description: "Build model from custom agents or generated clusters.",
            default_algorithm: ALGO_FLOCKING,
        },
    ]
}

pub fn algorithm_catalog() -> &'static [AlgorithmInfo] {
    &[
        AlgorithmInfo {
            id: ALGO_NONE,
            name: "No forces",
            description: "Integrate with zero external forces.",
            compatible_models: &[MODEL_RING, MODEL_LATTICE, MODEL_FROM_STATES],
        },
        AlgorithmInfo {
            id: ALGO_FLOCKING,
            name: "Flocking",
            description: "Cucker–Smale style flocking field with cohesion/alignment/separation.",
            compatible_models: &[MODEL_RING, MODEL_LATTICE, MODEL_FROM_STATES],
        },
        AlgorithmInfo {
            id: ALGO_FLOCKING_ALPHA,
            name: "Flocking alpha-lattice",
            description: "Alpha-lattice shaping + velocity consensus (Olfati-Saber style).",
            compatible_models: &[MODEL_RING, MODEL_LATTICE, MODEL_FROM_STATES],
        },
        AlgorithmInfo {
            id: ALGO_FORMATION_ECBF,
            name: "Fixed-time formation + ECBF",
            description: "Fixed-time formation tracking with robust ECBF obstacle avoidance.",
            compatible_models: &[
                MODEL_RING,
                MODEL_LATTICE,
                MODEL_QUADROTOR,
                MODEL_FROM_STATES,
            ],
        },
        AlgorithmInfo {
            id: ALGO_SAFE_FLOCKING_ALPHA,
            name: "Safe flocking (alpha + CBF-QP)",
            description: "Alpha-lattice nominal flocking filtered by CBF-QP (obstacles + inter-agent).",
            compatible_models: &[MODEL_RING, MODEL_LATTICE, MODEL_FROM_STATES],
        },
    ]
}

enum ModelKind {
    Particles(ParticleModel),
}

enum AlgorithmKind {
    None,
    Flocking(Flocking),
    FlockingAlpha(FlockingAlpha),
    FormationEcbf(FormationEcbf),
    SafeFlockingAlpha(SafeFlockingAlpha),
}

pub struct Engine {
    model_id: &'static str,
    algorithm_id: &'static str,
    model: ModelKind,
    algorithm: AlgorithmKind,
    plane_2d: bool,
}

impl Engine {
    pub fn new_builtin(model_id: &str, algorithm_id: Option<&str>) -> Result<Self, String> {
        let model_id = normalize_model_id(model_id)
            .ok_or_else(|| format!("unknown model id '{}'", model_id))?;
        let (model, default_algorithm_id) = build_model(model_id)?;
        let algorithm_id = match algorithm_id {
            Some(id) => normalize_algorithm_id(id)
                .ok_or_else(|| format!("unknown algorithm id '{}'", id))?,
            None => default_algorithm_id,
        };
        let algorithm = build_algorithm(algorithm_id, model_id)?;
        Ok(Self {
            model_id,
            algorithm_id,
            model,
            algorithm,
            plane_2d: false,
        })
    }

    pub fn new_custom(
        configs: Vec<BodyConfig>,
        dt: f64,
        algorithm_id: Option<&str>,
        plane_2d: bool,
    ) -> Result<Self, String> {
        let algorithm_id = algorithm_id.unwrap_or(ALGO_FLOCKING);
        let algorithm_id = normalize_algorithm_id(algorithm_id)
            .ok_or_else(|| format!("unknown algorithm id '{}'", algorithm_id))?;
        let algorithm = build_algorithm(algorithm_id, MODEL_FROM_STATES)?;
        let model = ParticleModel::new(configs, dt);
        Ok(Self {
            model_id: MODEL_FROM_STATES,
            algorithm_id,
            model: ModelKind::Particles(model),
            algorithm,
            plane_2d,
        })
    }

    pub fn new_from_states(
        states: Vec<f64>,
        dt: f64,
        algorithm_id: Option<&str>,
    ) -> Result<Self, String> {
        let model = ParticleModel::from_states(&states, dt, 0.0)
            .map_err(|e| format!("failed to create model from states: {}", e))?;
        let algorithm_id = match algorithm_id {
            Some(id) => normalize_algorithm_id(id)
                .ok_or_else(|| format!("unknown algorithm id '{}'", id))?,
            None => ALGO_FLOCKING,
        };
        let algorithm = build_algorithm(algorithm_id, MODEL_FROM_STATES)?;
        Ok(Self {
            model_id: MODEL_FROM_STATES,
            algorithm_id,
            model: ModelKind::Particles(model),
            algorithm,
            plane_2d: false,
        })
    }

    pub fn len(&self) -> usize {
        match &self.model {
            ModelKind::Particles(model) => model.len(),
        }
    }

    pub fn tick(&mut self) {
        match (&mut self.model, &mut self.algorithm) {
            (ModelKind::Particles(model), AlgorithmKind::Flocking(algo)) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                algo.apply(model, self.plane_2d);
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
            (ModelKind::Particles(model), AlgorithmKind::FlockingAlpha(algo)) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                algo.apply(model, self.plane_2d);
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
            (ModelKind::Particles(model), AlgorithmKind::FormationEcbf(algo)) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                algo.apply(model, self.plane_2d);
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
            (ModelKind::Particles(model), AlgorithmKind::SafeFlockingAlpha(algo)) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                algo.apply(model, self.plane_2d);
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
            (ModelKind::Particles(model), AlgorithmKind::None) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
        }
    }

    pub fn step(&mut self) {
        match &mut self.model {
            ModelKind::Particles(model) => {
                if self.plane_2d {
                    model.flatten_to_plane();
                }
                model.step();
                if self.plane_2d {
                    model.flatten_to_plane();
                }
            }
        }
    }

    pub fn positions_flat(&self) -> Vec<f32> {
        match &self.model {
            ModelKind::Particles(model) => {
                let positions = model.positions();
                let mut out = Vec::with_capacity(positions.len() * 3);
                for p in positions {
                    out.push(p.x as f32);
                    out.push(p.y as f32);
                    out.push(p.z as f32);
                }
                out
            }
        }
    }

    pub fn state_matrix_flat(&self) -> Vec<f32> {
        match &self.model {
            ModelKind::Particles(model) => {
                let states = model.state_matrix();
                let mut out = Vec::with_capacity(states.len() * 6);
                for row in states {
                    out.push(row[0] as f32);
                    out.push(row[1] as f32);
                    out.push(row[2] as f32);
                    out.push(row[3] as f32);
                    out.push(row[4] as f32);
                    out.push(row[5] as f32);
                }
                out
            }
        }
    }

    pub fn dt(&self) -> f64 {
        match &self.model {
            ModelKind::Particles(model) => model.dt(),
        }
    }

    pub fn groups(&self) -> Vec<u32> {
        match &self.model {
            ModelKind::Particles(model) => model.groups().iter().map(|g| *g as u32).collect(),
        }
    }

    pub fn set_plane_2d(&mut self, enabled: bool) { self.plane_2d = enabled; }

    pub fn set_algorithm(&mut self, algorithm_id: &str) -> Result<(), String> {
        let algorithm_id = normalize_algorithm_id(algorithm_id)
            .ok_or_else(|| format!("unknown algorithm id '{}'", algorithm_id))?;
        let algorithm = build_algorithm(algorithm_id, self.model_id)?;
        self.algorithm = algorithm;
        self.algorithm_id = algorithm_id;
        Ok(())
    }

    pub fn set_position(&mut self, index: usize, pos: Vector3<f64>) {
        match &mut self.model {
            ModelKind::Particles(model) => model.set_position(index, pos),
        }
    }

    pub fn set_velocity(&mut self, index: usize, vel: Vector3<f64>) {
        match &mut self.model {
            ModelKind::Particles(model) => model.set_velocity(index, vel),
        }
    }

    pub fn set_force(&mut self, index: usize, f: Vector3<f64>) {
        match &mut self.model {
            ModelKind::Particles(model) => model.set_force(index, f),
        }
    }

    pub fn set_uniform_force(&mut self, f: Vector3<f64>) {
        match &mut self.model {
            ModelKind::Particles(model) => model.set_uniform_force(f),
        }
    }

    pub fn set_flock_params(&mut self, params: FlockParams) -> Result<(), String> {
        match &mut self.algorithm {
            AlgorithmKind::Flocking(algo) => {
                algo.params = params;
                Ok(())
            }
            _ => Err("current algorithm does not support flocking params".to_string()),
        }
    }

    pub fn set_flock_alpha_params(&mut self, params: FlockAlphaParams) -> Result<(), String> {
        match &mut self.algorithm {
            AlgorithmKind::FlockingAlpha(algo) => {
                algo.params = params;
                Ok(())
            }
            _ => Err("current algorithm does not support flocking-alpha params".to_string()),
        }
    }

    pub fn set_formation_ecbf_params(&mut self, params: FormationEcbfParams) -> Result<(), String> {
        match &mut self.algorithm {
            AlgorithmKind::FormationEcbf(algo) => {
                algo.params = params;
                Ok(())
            }
            _ => Err("current algorithm does not support formation-ecbf params".to_string()),
        }
    }

    pub fn set_safe_flocking_alpha_params(&mut self, params: SafeFlockAlphaParams) -> Result<(), String> {
        match &mut self.algorithm {
            AlgorithmKind::SafeFlockingAlpha(algo) => {
                algo.params = params;
                Ok(())
            }
            _ => Err("current algorithm does not support safe-flocking-alpha params".to_string()),
        }
    }

    pub fn attitudes_flat(&self) -> Vec<f32> {
        match &self.algorithm {
            AlgorithmKind::FormationEcbf(algo) => algo.attitudes_flat(),
            _ => Vec::new(),
        }
    }

    pub fn reset_agent(&mut self, index: usize, pos: Vector3<f64>, vel: Vector3<f64>) {
        match &mut self.algorithm {
            AlgorithmKind::FormationEcbf(algo) => algo.reset_agent(index, pos, vel),
            _ => {}
        }
    }

    pub fn debug_states_flat(&self) -> Vec<f32> {
        match (&self.model, &self.algorithm) {
            (ModelKind::Particles(model), AlgorithmKind::SafeFlockingAlpha(algo)) => {
                let states = model.state_matrix();
                let dbg = algo.debug_flat();
                let n = states.len();
                if n == 0 {
                    return Vec::new();
                }
                // On the very first frame (before any tick/apply), the algorithm may not have
                // initialized its internal buffers yet. Avoid panicking on a short debug buffer.
                let dbg_ok = dbg.len() == n * 9;
                // Per-agent: 6 state + 9 debug.
                let mut out = Vec::with_capacity(n * 15);
                for i in 0..n {
                    let s = states[i];
                    out.push(s[0] as f32);
                    out.push(s[1] as f32);
                    out.push(s[2] as f32);
                    out.push(s[3] as f32);
                    out.push(s[4] as f32);
                    out.push(s[5] as f32);
                    if dbg_ok {
                        let base = i * 9;
                        out.push(dbg[base + 0]);
                        out.push(dbg[base + 1]);
                        out.push(dbg[base + 2]);
                        out.push(dbg[base + 3]);
                        out.push(dbg[base + 4]);
                        out.push(dbg[base + 5]);
                        out.push(dbg[base + 6]);
                        out.push(dbg[base + 7]);
                        out.push(dbg[base + 8]);
                    } else {
                        // unx,uny,unz, ux,uy,uz, slack, active, constraints
                        out.extend_from_slice(&[0.0; 9]);
                    }
                }
                out
            }
            _ => self.state_matrix_flat(),
        }
    }
}

fn normalize_model_id(id: &str) -> Option<&'static str> {
    match id {
        MODEL_RING => Some(MODEL_RING),
        MODEL_LATTICE => Some(MODEL_LATTICE),
        MODEL_QUADROTOR => Some(MODEL_QUADROTOR),
        MODEL_FROM_STATES => Some(MODEL_FROM_STATES),
        _ => None,
    }
}

fn normalize_algorithm_id(id: &str) -> Option<&'static str> {
    match id {
        ALGO_NONE => Some(ALGO_NONE),
        ALGO_FLOCKING => Some(ALGO_FLOCKING),
        ALGO_FLOCKING_ALPHA => Some(ALGO_FLOCKING_ALPHA),
        ALGO_FORMATION_ECBF => Some(ALGO_FORMATION_ECBF),
        ALGO_SAFE_FLOCKING_ALPHA => Some(ALGO_SAFE_FLOCKING_ALPHA),
        _ => None,
    }
}

fn build_algorithm(id: &'static str, model_id: &'static str) -> Result<AlgorithmKind, String> {
    let compatible = algorithm_catalog()
        .iter()
        .find(|a| a.id == id)
        .map(|a| a.compatible_models.contains(&model_id))
        .unwrap_or(false);

    if !compatible {
        return Err(format!("algorithm '{}' is not compatible with model '{}'", id, model_id));
    }

    match id {
        ALGO_NONE => Ok(AlgorithmKind::None),
        ALGO_FLOCKING => Ok(AlgorithmKind::Flocking(Flocking::new(FlockParams::default()))),
        ALGO_FLOCKING_ALPHA => Ok(AlgorithmKind::FlockingAlpha(FlockingAlpha::new(
            FlockAlphaParams::default(),
        ))),
        ALGO_FORMATION_ECBF => Ok(AlgorithmKind::FormationEcbf(FormationEcbf::new(
            FormationEcbfParams::default(),
        ))),
        ALGO_SAFE_FLOCKING_ALPHA => Ok(AlgorithmKind::SafeFlockingAlpha(SafeFlockingAlpha::new(
            SafeFlockAlphaParams::default(),
        ))),
        _ => Err(format!("unknown algorithm id '{}'", id)),
    }
}

fn build_model(model_id: &'static str) -> Result<(ModelKind, &'static str), String> {
    match model_id {
        MODEL_RING => {
            let configs = ring_demo_configs(DEMO_COUNT);
            let model = ParticleModel::new(configs, DEMO_DT);
            Ok((ModelKind::Particles(model), ALGO_FLOCKING))
        }
        MODEL_LATTICE => {
            let configs = lattice_demo_configs(3);
            let model = ParticleModel::new(configs, DEMO_DT);
            Ok((ModelKind::Particles(model), ALGO_FLOCKING))
        }
        MODEL_QUADROTOR => {
            let configs = quadrotor_demo_configs();
            let model = ParticleModel::new(configs, DEMO_DT);
            Ok((ModelKind::Particles(model), ALGO_FORMATION_ECBF))
        }
        MODEL_FROM_STATES => Err("from-states must be constructed with explicit state array".to_string()),
        _ => Err(format!("unknown model id '{}'", model_id)),
    }
}
