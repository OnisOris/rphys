use rphys::{BodyConfig, Point, Simulator};

pub enum FlockAlgorithm {
    Basic,
    WithGamma,
    WithObstacles,
}

pub struct FlockingBehavior {
    pub algorithm: FlockAlgorithm,
    pub eps: f64,
    pub r: f64,
    pub c_alpha1: f64,
    pub c_alpha2: f64,
    pub c_beta1: f64,
    pub c_beta2: f64,
    pub c_gamma1: f64,
    pub c_gamma2: f64,
    pub gamma: Option<(Vector3<f64>, Vector3<f64>)>,
    pub obstacles: Vec<Obstacle>, // center+radius
}
