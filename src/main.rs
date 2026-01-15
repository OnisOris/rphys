use nginphy::{Point, Simulator, BodyConfig};
use nalgebra::{Vector3};


fn main() {
    // Пример: точка стартует из (0,0,0), скорость (1,0,0).
    let mut p = Point::new(
        1.0,
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        true, // писать траекторию
        0.00, // коэффициент сопротивления
    );

    let dt = 0.01;
    let steps = 30;

    let force = Vector3::new(-10., 0.0, 0.0);

    for _ in 0..steps {
        p.step(force, dt);
    }

    println!("t = {:.3} s", p.time());
    println!("state = [x,y,z,vx,vy,vz] = {:?}", p.state());

    // Если нужна траектория, вот как взять:
    let traj = p.trajectory();
    println!("trajectory len = {}", traj.len());
    println!("last point (x,y,z,vx,vy,vz,t) = {:?}", traj.last().unwrap());
    println!("{:#?}", traj);

    // Демонстрация группового симулятора на 2 телах
    let mut sim = Simulator::new(
        &[
            BodyConfig { mass: 1.0, state: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], drag_coefficient: 0.0, trajectory_write: true, group: 0 },
            BodyConfig { mass: 2.0, state: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], drag_coefficient: 0.0, trajectory_write: false, group: 0 },
        ],
        0.01,
    );
    sim.set_force(0, Vector3::new(-10.0, 0.0, 0.0));
    sim.set_force(1, Vector3::new(0.0, 0.0, 0.0));
    for _ in 0..30 { sim.step(); }
    println!("sim t = {:.3} s", sim.time());
    let states = sim.state_matrix();
    println!("states (Nx6) = {:?}", states);
}
