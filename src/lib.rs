use nalgebra::{SVector, Vector3};
pub mod sim;
pub use sim::{BodyConfig, BodySnapshot, ForceField, Simulator};

#[cfg(target_arch = "wasm32")]
pub mod wasm;

/// Материальная точка со state = [x, y, z, vx, vy, vz]
#[derive(Debug, Clone)]
pub struct Point {
    mass: f64,
    drag_coefficient: f64,
    state: SVector<f64, 6>,
    time: f64,
    /// Если true, пишем траекторию: [x, y, z, vx, vy, vz, t]
    trajectory_write: bool,
    trajectory: Vec<[f64; 7]>,
}

impl Point {
    /// Создание точки.
    /// position: [x, y, z, vx, vy, vz]
    pub fn new(
        mass: f64,
        position: [f64; 6],
        trajectory_write: bool,
        drag_coefficient: f64,
    ) -> Self {
        let mut p = Self {
            mass,
            drag_coefficient,
            state: SVector::<f64, 6>::from_row_slice(&position),
            time: 0.0,
            trajectory_write,
            trajectory: Vec::new(),
        };
        if p.trajectory_write {
            p.push_traj();
        }
        p
    }

    /// Скорость [vx, vy, vz].
    fn velocity(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(3).into()
    }

    /// Один шаг интегрирования (RK4).
    /// force: внешняя сила [Fx, Fy, Fz], dt: шаг времени.
    pub fn step(&mut self, force: Vector3<f64>, dt: f64) {
        // Сопротивление считаем по текущей скорости (как в вашем коде, b константно в пределах шага)
        let v = self.velocity();
        let drag_force = -self.drag_coefficient * v;
        let total_force = force + drag_force;
        let acc = total_force / self.mass; // [ax, ay, az]

        // Производная для состояния: [vx, vy, vz, ax, ay, az]
        let f = |s: &SVector<f64, 6>| -> SVector<f64, 6> {
            SVector::<f64, 6>::from_row_slice(&[s[3], s[4], s[5], acc[0], acc[1], acc[2]])
        };

        // RK4
        let k1 = dt * f(&self.state);
        let k2 = dt * f(&(self.state + 0.5 * k1));
        let k3 = dt * f(&(self.state + 0.5 * k2));
        let k4 = dt * f(&(self.state + k3));

        self.state += (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        self.time += dt;

        if self.trajectory_write {
            self.push_traj();
        }
    }

    /// Получить траекторию как срез (если запись включена).
    /// Возвращаем &[[f64; 7]], чтобы не раскрывать тип контейнера.
    pub fn trajectory(&self) -> &[[f64; 7]] {
        &self.trajectory
    }

    /// Текущее модельное время.
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Полное состояние [x, y, z, vx, vy, vz].
    pub fn state(&self) -> &SVector<f64, 6> {
        &self.state
    }

    fn push_traj(&mut self) {
        self.trajectory.push([
            self.state[0],
            self.state[1],
            self.state[2],
            self.state[3],
            self.state[4],
            self.state[5],
            self.time,
        ]);
    }

    /// Снимок состояния в виде массива [x, y, z, vx, vy, vz].
    pub fn state_array(&self) -> [f64; 6] {
        let mut out = [0.0; 6];
        out.copy_from_slice(self.state.as_slice());
        out
    }

    pub fn mass(&self) -> f64 { self.mass }
    pub fn drag_coefficient(&self) -> f64 { self.drag_coefficient }
    pub fn trajectory_write(&self) -> bool { self.trajectory_write }

    /// Преобразовать точку в конфигурацию для симулятора с указанной группой.
    pub fn to_body_config(&self, group: usize) -> BodyConfig {
        BodyConfig {
            mass: self.mass,
            state: self.state_array(),
            drag_coefficient: self.drag_coefficient,
            trajectory_write: self.trajectory_write,
            group,
        }
    }
}
