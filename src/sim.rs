use nalgebra::Vector3;

/// Начальная конфигурация одного тела для симулятора.
#[derive(Debug, Clone)]
pub struct BodyConfig {
    pub mass: f64,
    /// state = [x, y, z, vx, vy, vz]
    pub state: [f64; 6],
    pub drag_coefficient: f64,
    /// Записывать ли траекторию этого тела
    pub trajectory_write: bool,
    /// Номер группы (для удобства массового задания сил)
    pub group: usize,
}

/// Полная информация об одном теле в текущий момент времени.
#[derive(Debug, Clone)]
pub struct BodySnapshot {
    pub mass: f64,
    pub state: [f64; 6],
    pub drag_coefficient: f64,
    pub trajectory_write: bool,
    pub group: usize,
    pub force: [f64; 3],
}

impl BodyConfig {
    pub fn new(mass: f64, state: [f64; 6]) -> Self {
        Self {
            mass,
            state,
            drag_coefficient: 0.0,
            trajectory_write: false,
            group: 0,
        }
    }
}

/// Поле сил для всех тел. Заполняет вектор сил для каждого индекса тела.
pub trait ForceField: Sync {
    fn force_all(
        &self,
        t: f64,
        x: &[Vector3<f64>],
        v: &[Vector3<f64>],
        out: &mut [Vector3<f64>],
    );
}

// Удобство: можно передать замыкание вида Fn(t, x, v, out)
impl<F> ForceField for F
where
    F: Fn(f64, &[Vector3<f64>], &[Vector3<f64>], &mut [Vector3<f64>]) + Sync,
{
    fn force_all(
        &self,
        t: f64,
        x: &[Vector3<f64>],
        v: &[Vector3<f64>],
        out: &mut [Vector3<f64>],
    ) {
        (self)(t, x, v, out)
    }
}

/// Групповой симулятор для множества точек с трансляционным состоянием.
/// Хранит массивы позиций/скоростей (SoA), массы, коэффициенты сопротивления и настройку записи траекторий.
#[derive(Debug)]
pub struct Simulator {
    x: Vec<Vector3<f64>>,         // позиции
    v: Vec<Vector3<f64>>,         // скорости
    mass: Vec<f64>,               // массы
    drag: Vec<f64>,               // коэффициенты сопротивления
    groups: Vec<usize>,           // группа для каждого тела
    forces: Vec<Vector3<f64>>,    // внешние силы (на объект), по умолчанию нули
    traj: Vec<Option<Vec<[f64; 7]>>>, // траектории по объектам (Option, чтобы не аллоцировать лишнее)
    time: f64,                    // глобальное модельное время
    dt: f64,                      // шаг по времени по умолчанию

    // Рабочие буферы для RK4, чтобы не аллоцировать на каждом шаге
    k1x: Vec<Vector3<f64>>, k1v: Vec<Vector3<f64>>,
    k2x: Vec<Vector3<f64>>, k2v: Vec<Vector3<f64>>,
    k3x: Vec<Vector3<f64>>, k3v: Vec<Vector3<f64>>,
    k4x: Vec<Vector3<f64>>, k4v: Vec<Vector3<f64>>,
    tmp_x: Vec<Vector3<f64>>, tmp_v: Vec<Vector3<f64>>,
    fx: Vec<Vector3<f64>>,       // вектор сил-текущая фаза
}

impl Simulator {
    pub fn new(configs: &[BodyConfig], dt: f64) -> Self {
        let n = configs.len();
        let mut x = Vec::with_capacity(n);
        let mut v = Vec::with_capacity(n);
        let mut mass = Vec::with_capacity(n);
        let mut drag = Vec::with_capacity(n);
        let mut groups = Vec::with_capacity(n);
        let mut forces = Vec::with_capacity(n);
        let mut traj = Vec::with_capacity(n);

        for c in configs {
            x.push(Vector3::new(c.state[0], c.state[1], c.state[2]));
            v.push(Vector3::new(c.state[3], c.state[4], c.state[5]));
            mass.push(c.mass);
            drag.push(c.drag_coefficient);
            groups.push(c.group);
            forces.push(Vector3::new(0.0, 0.0, 0.0));
            traj.push(if c.trajectory_write { Some(Vec::new()) } else { None });
        }

        let zero = || vec![Vector3::new(0.0, 0.0, 0.0); n];
        let tmp = || vec![Vector3::new(0.0, 0.0, 0.0); n];

        Self {
            x,
            v,
            mass,
            drag,
            groups,
            forces,
            traj,
            time: 0.0,
            dt,
            k1x: zero(), k1v: zero(),
            k2x: zero(), k2v: zero(),
            k3x: zero(), k3v: zero(),
            k4x: zero(), k4v: zero(),
            tmp_x: tmp(), tmp_v: tmp(),
            fx: vec![Vector3::new(0.0, 0.0, 0.0); n],
        }
    }

    pub fn len(&self) -> usize { self.x.len() }
    pub fn is_empty(&self) -> bool { self.x.is_empty() }
    pub fn time(&self) -> f64 { self.time }
    pub fn dt(&self) -> f64 { self.dt }
    pub fn set_dt(&mut self, dt: f64) { self.dt = dt; }
    pub(crate) fn groups(&self) -> &[usize] { &self.groups }

    /// Получить матрицу состояний Nx6 (x,y,z,vx,vy,vz)
    pub fn state_matrix(&self) -> Vec<[f64; 6]> {
        let n = self.len();
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            out.push([
                self.x[i].x, self.x[i].y, self.x[i].z,
                self.v[i].x, self.v[i].y, self.v[i].z,
            ]);
        }
        out
    }

    pub(crate) fn positions(&self) -> &[Vector3<f64>] { &self.x }

    pub(crate) fn velocities(&self) -> &[Vector3<f64>] { &self.v }

    pub(crate) fn set_position(&mut self, i: usize, pos: Vector3<f64>) { self.x[i] = pos; }

    pub(crate) fn set_velocity(&mut self, i: usize, vel: Vector3<f64>) { self.v[i] = vel; }

    /// Ссылка на траекторию i-го тела, если запись включена
    pub fn trajectory_of(&self, i: usize) -> Option<&[[f64; 7]]> {
        self.traj[i].as_deref()
    }

    /// Сформировать удобный снимок по всем телам (для UI/отладочного вывода).
    pub fn body_snapshots(&self) -> Vec<BodySnapshot> {
        let n = self.len();
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            out.push(BodySnapshot {
                mass: self.mass[i],
                state: [
                    self.x[i].x, self.x[i].y, self.x[i].z,
                    self.v[i].x, self.v[i].y, self.v[i].z,
                ],
                drag_coefficient: self.drag[i],
                trajectory_write: self.traj[i].is_some(),
                group: self.groups[i],
                force: [self.forces[i].x, self.forces[i].y, self.forces[i].z],
            });
        }
        out
    }

    /// Назначить внешнюю силу для объекта i
    pub fn set_force(&mut self, i: usize, f: Vector3<f64>) {
        self.forces[i] = f;
    }

    /// Массовое назначение силы по группе
    pub fn set_group_force(&mut self, group: usize, f: Vector3<f64>) {
        for (i, g) in self.groups.iter().copied().enumerate() {
            if g == group { self.forces[i] = f; }
        }
    }

    /// Один шаг RK4, используя сохранённые в self.forces (константные во времени на шаг) внешние силы.
    pub fn step(&mut self) {
        let n = self.len();
        if n == 0 { return; }
        let dt = self.dt;

        // k1
        self.fx.copy_from_slice(&self.forces);
        for i in 0..n {
            self.k1x[i] = self.v[i];
            let acc = (self.fx[i] - self.drag[i] * self.v[i]) / self.mass[i];
            self.k1v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k1x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k1v[i];
        }

        // k2
        self.fx.copy_from_slice(&self.forces);
        for i in 0..n {
            self.k2x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k2v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k2x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k2v[i];
        }

        // k3
        self.fx.copy_from_slice(&self.forces);
        for i in 0..n {
            self.k3x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k3v[i] = acc;
            self.tmp_x[i] = self.x[i] + dt * self.k3x[i];
            self.tmp_v[i] = self.v[i] + dt * self.k3v[i];
        }

        // k4
        self.fx.copy_from_slice(&self.forces);
        for i in 0..n {
            self.k4x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k4v[i] = acc;
        }

        // Обновление состояния
        for i in 0..n {
            self.x[i] += dt * (self.k1x[i] + 2.0 * self.k2x[i] + 2.0 * self.k3x[i] + self.k4x[i]) / 6.0;
            self.v[i] += dt * (self.k1v[i] + 2.0 * self.k2v[i] + 2.0 * self.k3v[i] + self.k4v[i]) / 6.0;
        }

        self.time += dt;

        // Запись траекторий, если включено
        for i in 0..n {
            if let Some(t) = &mut self.traj[i] {
                t.push([
                    self.x[i].x, self.x[i].y, self.x[i].z,
                    self.v[i].x, self.v[i].y, self.v[i].z,
                    self.time,
                ]);
            }
        }
    }

    /// Один шаг RK4 с внешним полем сил field.
    pub fn step_with_field<F: ForceField>(&mut self, field: &F) {
        let n = self.len();
        if n == 0 { return; }
        let dt = self.dt;

        // k1
        field.force_all(self.time, &self.x, &self.v, &mut self.fx);
        for i in 0..n {
            self.k1x[i] = self.v[i];
            let acc = (self.fx[i] - self.drag[i] * self.v[i]) / self.mass[i];
            self.k1v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k1x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k1v[i];
        }

        // k2
        field.force_all(self.time + 0.5 * dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        for i in 0..n {
            self.k2x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k2v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k2x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k2v[i];
        }

        // k3
        field.force_all(self.time + 0.5 * dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        for i in 0..n {
            self.k3x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k3v[i] = acc;
            self.tmp_x[i] = self.x[i] + dt * self.k3x[i];
            self.tmp_v[i] = self.v[i] + dt * self.k3v[i];
        }

        // k4
        field.force_all(self.time + dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        for i in 0..n {
            self.k4x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k4v[i] = acc;
        }

        // Обновление состояния
        for i in 0..n {
            self.x[i] += dt * (self.k1x[i] + 2.0 * self.k2x[i] + 2.0 * self.k3x[i] + self.k4x[i]) / 6.0;
            self.v[i] += dt * (self.k1v[i] + 2.0 * self.k2v[i] + 2.0 * self.k3v[i] + self.k4v[i]) / 6.0;
        }

        self.time += dt;

        // Запись траекторий, если включено
        for i in 0..n {
            if let Some(t) = &mut self.traj[i] {
                t.push([
                    self.x[i].x, self.x[i].y, self.x[i].z,
                    self.v[i].x, self.v[i].y, self.v[i].z,
                    self.time,
                ]);
            }
        }
    }

    /// Параллельный шаг RK4 (только если включена фича parallel).
    #[cfg(feature = "parallel")]
    pub fn step_par_with_field<F: ForceField>(&mut self, field: &F) {
        use rayon::prelude::*;
        let n = self.len();
        if n == 0 { return; }
        let dt = self.dt;

        // k1
        field.force_all(self.time, &self.x, &self.v, &mut self.fx);
        (0..n).into_par_iter().for_each(|i| {
            self.k1x[i] = self.v[i];
            let acc = (self.fx[i] - self.drag[i] * self.v[i]) / self.mass[i];
            self.k1v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k1x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k1v[i];
        });

        // k2
        field.force_all(self.time + 0.5 * dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        (0..n).into_par_iter().for_each(|i| {
            self.k2x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k2v[i] = acc;
            self.tmp_x[i] = self.x[i] + 0.5 * dt * self.k2x[i];
            self.tmp_v[i] = self.v[i] + 0.5 * dt * self.k2v[i];
        });

        // k3
        field.force_all(self.time + 0.5 * dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        (0..n).into_par_iter().for_each(|i| {
            self.k3x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k3v[i] = acc;
            self.tmp_x[i] = self.x[i] + dt * self.k3x[i];
            self.tmp_v[i] = self.v[i] + dt * self.k3v[i];
        });

        // k4
        field.force_all(self.time + dt, &self.tmp_x, &self.tmp_v, &mut self.fx);
        (0..n).into_par_iter().for_each(|i| {
            self.k4x[i] = self.tmp_v[i];
            let acc = (self.fx[i] - self.drag[i] * self.tmp_v[i]) / self.mass[i];
            self.k4v[i] = acc;
        });

        // Обновление состояния
        (0..n).into_par_iter().for_each(|i| {
            self.x[i] += dt * (self.k1x[i] + 2.0 * self.k2x[i] + 2.0 * self.k3x[i] + self.k4x[i]) / 6.0;
            self.v[i] += dt * (self.k1v[i] + 2.0 * self.k2v[i] + 2.0 * self.k3v[i] + self.k4v[i]) / 6.0;
        });

        self.time += dt;

        // Запись траекторий
        (0..n).into_par_iter().for_each(|i| {
            if let Some(t) = &mut self.traj[i] {
                t.push([
                    self.x[i].x, self.x[i].y, self.x[i].z,
                    self.v[i].x, self.v[i].y, self.v[i].z,
                    self.time,
                ]);
            }
        });
    }
}
