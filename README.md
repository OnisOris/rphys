# rphys
Физический движок написан на Rust.

## Установка wasm-pack

```
cargo install wasm-pack
```

## Демо в браузере (WASM + JS)

Собрать rphys:
```
cargo build
```

Собрать wasm-пакет:
```
wasm-pack build --target web --out-dir www/pkg
```

Запустить статический сервер:
```
python3 -m http.server --directory www
```

Откройте http://localhost:8000 в браузере.

Альтернатива: dev-сервер на Rust:
```
cargo run --bin devserver -- --dir www --port 8000
```

## Чтение .pb траекторий

`pb_reader` содержит CLI-инструмент, который читает записи траекторий в формате
protobuf и строит графики через Matplotlib.

Запуск через uv:
```
cd pb_reader
uv run rphys-pb ./files/rphys-history-1769409283410.pb --x t --y x --agent 0
```

Основные опции:
- `--agent N` — выбрать агента (индекс с 0).
- `--all` — построить график для всех агентов.
- `--x` / `--y` — оси (`t, x, y, z, vx, vy, vz`).
- `--list-axes` — показать доступные оси.
- `--out plot.png` — сохранить график в файл.
- `--backend QtAgg` — принудительно использовать интерактивный backend.

Примеры:
```
# скорость vx от времени для агента 0
uv run rphys-pb ./files/rphys-history-1769409283410.pb --x t --y vx --agent 0

# координата y от x для агента 2
uv run rphys-pb ./files/rphys-history-1769409283410.pb --x x --y y --agent 2

# графики x(t) для всех агентов
uv run rphys-pb ./files/rphys-history-1769409283410.pb --x t --y x --all
```

Метрики флокинга (все графики сразу):
- `C(t)` — связность,
- `R(t)` — радиус когезии,
- `E~(t)` — нормированная энергия отклонения,
- `K~(t)` — нормированная несогласованность скоростей.

```
uv run rphys-pb ./files/rphys-history-1769409283410.pb --metrics
```

Если в .pb нет параметров алгоритма, укажите радиус соседства и желаемую
дистанцию вручную:
```
uv run rphys-pb ./files/rphys-history-1769409283410.pb --metrics \
  --neighbor-radius 2.6 --desired-distance 0.9
```
