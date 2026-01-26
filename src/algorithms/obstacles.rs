use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObstaclePoly {
    /// p(t) = a2 * t^2 + a1 * t + a0
    pub a2: [f64; 3],
    pub a1: [f64; 3],
    pub a0: [f64; 3],
    pub d: f64,
}

impl ObstaclePoly {
    pub fn pos(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            self.a2[0] * t * t + self.a1[0] * t + self.a0[0],
            self.a2[1] * t * t + self.a1[1] * t + self.a0[1],
            self.a2[2] * t * t + self.a1[2] * t + self.a0[2],
        )
    }

    pub fn vel(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            2.0 * self.a2[0] * t + self.a1[0],
            2.0 * self.a2[1] * t + self.a1[1],
            2.0 * self.a2[2] * t + self.a1[2],
        )
    }

    pub fn acc(&self) -> Vector3<f64> {
        Vector3::new(2.0 * self.a2[0], 2.0 * self.a2[1], 2.0 * self.a2[2])
    }
}

pub fn paper_obstacles() -> Vec<ObstaclePoly> {
    vec![
        ObstaclePoly {
            a2: [0.0, 0.0, 0.0],
            a1: [0.0, 0.0, 0.0],
            a0: [47.0, 86.0, 10.0],
            d: 5.0,
        },
        ObstaclePoly {
            a2: [0.0, 0.0, 0.0],
            a1: [0.0, 0.0, 0.0],
            a0: [52.0, 78.0, 9.0],
            d: 4.0,
        },
        ObstaclePoly {
            a2: [0.0, 0.0, 0.0],
            a1: [0.0, 0.0, 0.0],
            a0: [43.0, 82.0, 61.5],
            d: 5.0,
        },
        ObstaclePoly {
            a2: [0.0, 0.0, 0.0],
            a1: [0.0, 0.0, 0.0],
            a0: [49.0, 75.0, 60.5],
            d: 5.5,
        },
        // moving obstacle: p = [95 - 0.06t, 15 + 0.001 t^2, 100 - 0.089 t]
        ObstaclePoly {
            a2: [0.0, 0.001, 0.0],
            a1: [-0.06, 0.0, -0.089],
            a0: [95.0, 15.0, 100.0],
            d: 3.0,
        },
        ObstaclePoly {
            a2: [0.0, 0.0, 0.0],
            a1: [0.0, 0.0, 0.0],
            a0: [69.0, 83.0, 124.5],
            d: 6.0,
        },
    ]
}

