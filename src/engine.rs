use crate::algorithms::flocking::{FlockParams, Flocking};
use crate::models::particles::{lattice_demo_configs, ring_demo_configs, ParticleModel, DEMO_COUNT, DEMO_DT};
use crate::BodyConfig;
use nalgebra::Vector3;

pub const MODEL_RING: &str = "ring-swarm";
pub const MODEL_LATTICE: &str = "lattice-swarm";
pub const MODEL_FROM_STATES: &str = "from-states";

pub const ALGO_NONE: &str = "none";
pub const ALGO_FLOCKING: &str = "flocking";

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
    ]
}

enum ModelKind {
    Particles(ParticleModel),
}

enum AlgorithmKind {
    None,
    Flocking(Flocking),
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
}

fn normalize_model_id(id: &str) -> Option<&'static str> {
    match id {
        MODEL_RING => Some(MODEL_RING),
        MODEL_LATTICE => Some(MODEL_LATTICE),
        MODEL_FROM_STATES => Some(MODEL_FROM_STATES),
        _ => None,
    }
}

fn normalize_algorithm_id(id: &str) -> Option<&'static str> {
    match id {
        ALGO_NONE => Some(ALGO_NONE),
        ALGO_FLOCKING => Some(ALGO_FLOCKING),
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
        MODEL_FROM_STATES => Err("from-states must be constructed with explicit state array".to_string()),
        _ => Err(format!("unknown model id '{}'", model_id)),
    }
}
