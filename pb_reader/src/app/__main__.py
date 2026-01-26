from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DEFAULT_FIELDS = ["x", "y", "z", "vx", "vy", "vz"]
PREFERRED_INTERACTIVE_BACKEND = "QtAgg"
AXIS_LABELS_RU_SI = {
    "t": "время, с",
    "x": "координата x, м",
    "y": "координата y, м",
    "z": "координата z, м",
    "vx": "скорость vx, м/с",
    "vy": "скорость vy, м/с",
    "vz": "скорость vz, м/с",
}
FLOCK_PARAM_FALLBACKS = {
    "neighbor_radius": 2.6,
    "separation_radius": 0.9,
}
FLOCK_ALPHA_PARAM_FALLBACKS = {
    "neighbor_radius": 2.6,
    "desired_distance": 1.4,
}

@dataclass(frozen=True)
class ObstaclePoly:
    """p(t) = a2 * t^2 + a1 * t + a0"""

    a2: np.ndarray
    a1: np.ndarray
    a0: np.ndarray
    d: float

    def pos(self, t: np.ndarray) -> np.ndarray:
        t = np.asarray(t, dtype=np.float64)
        t2 = t * t
        return self.a2[None, :] * t2[:, None] + self.a1[None, :] * t[:, None] + self.a0[None, :]


def paper_obstacles() -> list[ObstaclePoly]:
    return [
        ObstaclePoly(
            a2=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a1=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a0=np.array([47.0, 86.0, 10.0], dtype=np.float64),
            d=5.0,
        ),
        ObstaclePoly(
            a2=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a1=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a0=np.array([52.0, 78.0, 9.0], dtype=np.float64),
            d=4.0,
        ),
        ObstaclePoly(
            a2=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a1=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a0=np.array([43.0, 82.0, 61.5], dtype=np.float64),
            d=5.0,
        ),
        ObstaclePoly(
            a2=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a1=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a0=np.array([49.0, 75.0, 60.5], dtype=np.float64),
            d=5.5,
        ),
        # moving obstacle: p = [95 - 0.06t, 15 + 0.001 t^2, 100 - 0.089 t]
        ObstaclePoly(
            a2=np.array([0.0, 0.001, 0.0], dtype=np.float64),
            a1=np.array([-0.06, 0.0, -0.089], dtype=np.float64),
            a0=np.array([95.0, 15.0, 100.0], dtype=np.float64),
            d=3.0,
        ),
        ObstaclePoly(
            a2=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a1=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            a0=np.array([69.0, 83.0, 124.5], dtype=np.float64),
            d=6.0,
        ),
    ]


@dataclass
class RecordMeta:
    version: int = 0
    created_at: str = ""
    dt: float = math.nan
    stride: int = 1
    max_frames: int = 0
    model_id: str = ""
    algorithm_id: str = ""
    algorithm_params: dict[str, float] = field(default_factory=dict)
    plane2d: bool = False
    agent_count: int = 0
    fields: list[str] = field(default_factory=list)
    group_colors: dict[int, str] = field(default_factory=dict)
    groups: list[int] = field(default_factory=list)


@dataclass
class Recording:
    meta: RecordMeta
    frame_count: int
    agent_count: int
    fields: list[str]
    states: np.ndarray
    time: np.ndarray


class ProtoError(ValueError):
    pass


def read_varint(data: memoryview, offset: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while True:
        if offset >= len(data):
            raise ProtoError("varint truncated")
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, offset
        shift += 7
        if shift > 63:
            raise ProtoError("varint too long")


def skip_field(data: memoryview, offset: int, wire_type: int) -> int:
    if wire_type == 0:
        _, offset = read_varint(data, offset)
        return offset
    if wire_type == 1:
        return offset + 8
    if wire_type == 2:
        length, offset = read_varint(data, offset)
        return offset + length
    if wire_type == 5:
        return offset + 4
    raise ProtoError(f"unsupported wire type {wire_type}")


def parse_group_color(payload: memoryview) -> tuple[int | None, str | None]:
    group = None
    color = None
    offset = 0
    while offset < len(payload):
        tag, offset = read_varint(payload, offset)
        field = tag >> 3
        wire_type = tag & 7
        if field == 1 and wire_type == 0:
            group, offset = read_varint(payload, offset)
        elif field == 2 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            color = payload[offset : offset + length].tobytes().decode("utf-8", "replace")
            offset += length
        else:
            offset = skip_field(payload, offset, wire_type)
    return group, color


def parse_algorithm_param(payload: memoryview) -> tuple[str | None, float | None]:
    key = None
    value = None
    offset = 0
    while offset < len(payload):
        tag, offset = read_varint(payload, offset)
        field = tag >> 3
        wire_type = tag & 7
        if field == 1 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            key = payload[offset : offset + length].tobytes().decode(
                "utf-8", "replace"
            )
            offset += length
        elif field == 2 and wire_type == 1:
            value = struct.unpack_from("<d", payload, offset)[0]
            offset += 8
        else:
            offset = skip_field(payload, offset, wire_type)
    return key, value


def parse_packed_varints(payload: memoryview) -> list[int]:
    values = []
    offset = 0
    while offset < len(payload):
        value, offset = read_varint(payload, offset)
        values.append(value)
    return values


def parse_record_meta(payload: memoryview) -> RecordMeta:
    meta = RecordMeta()
    offset = 0
    while offset < len(payload):
        tag, offset = read_varint(payload, offset)
        field = tag >> 3
        wire_type = tag & 7
        if field == 1 and wire_type == 0:
            meta.version, offset = read_varint(payload, offset)
        elif field == 2 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            meta.created_at = payload[offset : offset + length].tobytes().decode(
                "utf-8", "replace"
            )
            offset += length
        elif field == 3 and wire_type == 1:
            meta.dt = struct.unpack_from("<d", payload, offset)[0]
            offset += 8
        elif field == 4 and wire_type == 0:
            meta.stride, offset = read_varint(payload, offset)
        elif field == 5 and wire_type == 0:
            meta.max_frames, offset = read_varint(payload, offset)
        elif field == 6 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            meta.model_id = payload[offset : offset + length].tobytes().decode(
                "utf-8", "replace"
            )
            offset += length
        elif field == 7 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            meta.algorithm_id = payload[offset : offset + length].tobytes().decode(
                "utf-8", "replace"
            )
            offset += length
        elif field == 8 and wire_type == 0:
            value, offset = read_varint(payload, offset)
            meta.plane2d = bool(value)
        elif field == 9 and wire_type == 0:
            meta.agent_count, offset = read_varint(payload, offset)
        elif field == 10 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            field_name = payload[offset : offset + length].tobytes().decode(
                "utf-8", "replace"
            )
            meta.fields.append(field_name)
            offset += length
        elif field == 11 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            group, color = parse_group_color(payload[offset : offset + length])
            if group is not None and color is not None:
                meta.group_colors[group] = color
            offset += length
        elif field == 12 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            meta.groups.extend(parse_packed_varints(payload[offset : offset + length]))
            offset += length
        elif field == 13 and wire_type == 2:
            length, offset = read_varint(payload, offset)
            key, value = parse_algorithm_param(payload[offset : offset + length])
            if key is not None and value is not None:
                meta.algorithm_params[key] = value
            offset += length
        else:
            offset = skip_field(payload, offset, wire_type)
    return meta


def parse_recording(data: bytes) -> tuple[RecordMeta, int, np.ndarray]:
    meta_payload = None
    frame_count = 0
    states_payload = None
    view = memoryview(data)
    offset = 0
    while offset < len(view):
        tag, offset = read_varint(view, offset)
        field = tag >> 3
        wire_type = tag & 7
        if field == 1 and wire_type == 2:
            length, offset = read_varint(view, offset)
            meta_payload = view[offset : offset + length]
            offset += length
        elif field == 2 and wire_type == 0:
            frame_count, offset = read_varint(view, offset)
        elif field == 3 and wire_type == 2:
            length, offset = read_varint(view, offset)
            states_payload = view[offset : offset + length]
            offset += length
        else:
            offset = skip_field(view, offset, wire_type)

    meta = parse_record_meta(meta_payload) if meta_payload else RecordMeta()

    if states_payload is None:
        raise ProtoError("states payload missing")
    if len(states_payload) % 4 != 0:
        raise ProtoError("states payload length is not a multiple of 4 bytes")
    states = np.frombuffer(states_payload, dtype="<f4")
    return meta, frame_count, states


def build_recording(path: Path) -> Recording:
    meta, frame_count, states = parse_recording(path.read_bytes())
    if meta.fields:
        fields = [f.strip().lower() for f in meta.fields if f.strip()]
    else:
        fields = list(DEFAULT_FIELDS)
    field_count = len(fields)
    if field_count == 0:
        raise ProtoError("no fields in recording")

    total_values = int(states.size)
    agent_count = meta.agent_count
    if agent_count <= 0 and meta.groups:
        agent_count = len(meta.groups)
    if agent_count <= 0 and frame_count > 0:
        agent_count = total_values // (frame_count * field_count)
    if frame_count <= 0 and agent_count > 0:
        frame_count = total_values // (agent_count * field_count)
    if agent_count <= 0 or frame_count <= 0:
        raise ProtoError("unable to infer agent/frame counts")

    expected = frame_count * agent_count * field_count
    if total_values < expected:
        raise ProtoError("states payload is truncated")
    if total_values > expected:
        print(
            f"warning: states payload has {total_values} values, "
            f"expected {expected}; extra values will be ignored",
            file=sys.stderr,
        )
        states = states[:expected]

    states = states.reshape(frame_count, agent_count, field_count)

    dt = meta.dt if math.isfinite(meta.dt) and meta.dt > 0 else 1.0
    stride = meta.stride if meta.stride > 0 else 1
    if not math.isfinite(meta.dt) or meta.dt <= 0:
        print("warning: dt missing; using 1.0 с", file=sys.stderr)
    if meta.stride <= 0:
        print("warning: stride missing; using 1", file=sys.stderr)
    time = np.arange(frame_count, dtype=np.float64) * dt * stride

    return Recording(meta, frame_count, agent_count, fields, states, time)


def normalize_axis(name: str) -> str:
    axis = name.strip().lower()
    if axis == "time":
        axis = "t"
    return axis


def axis_label(axis: str) -> str:
    axis = normalize_axis(axis)
    label = AXIS_LABELS_RU_SI.get(axis)
    if label is None:
        raise ProtoError(f"unknown axis '{axis}'")
    return label


def list_axes(fields: list[str]) -> list[str]:
    axes = ["t"]
    for field in fields:
        if normalize_axis(field) in AXIS_LABELS_RU_SI:
            axes.append(field)
    return axes


def axis_series(recording: Recording, axis: str, agent: int) -> np.ndarray:
    axis = normalize_axis(axis)
    if axis == "t":
        return recording.time
    if axis not in recording.fields:
        raise ProtoError(f"axis '{axis}' not found in recording fields")
    index = recording.fields.index(axis)
    return recording.states[:, agent, index]


def field_index(fields: list[str], name: str) -> int | None:
    try:
        return fields.index(name)
    except ValueError:
        return None


def extract_state_vectors(recording: Recording) -> tuple[np.ndarray, np.ndarray, int]:
    fields = recording.fields
    x_idx = field_index(fields, "x")
    y_idx = field_index(fields, "y")
    z_idx = field_index(fields, "z")
    vx_idx = field_index(fields, "vx")
    vy_idx = field_index(fields, "vy")
    vz_idx = field_index(fields, "vz")

    if x_idx is None or y_idx is None:
        raise ProtoError("позиции x/y отсутствуют в записи")
    if vx_idx is None or vy_idx is None:
        raise ProtoError("скорости vx/vy отсутствуют в записи")

    is_2d = recording.meta.plane2d or z_idx is None or vz_idx is None
    if not recording.meta.plane2d and (z_idx is None or vz_idx is None):
        print(
            "Предупреждение: нет полных данных по оси z, "
            "метрики считаются в 2D",
            file=sys.stderr,
        )

    pos_fields = [x_idx, y_idx]
    vel_fields = [vx_idx, vy_idx]
    if not is_2d:
        pos_fields.append(z_idx)
        vel_fields.append(vz_idx)

    positions = recording.states[:, :, pos_fields]
    velocities = recording.states[:, :, vel_fields]
    dim = 2 if is_2d else 3
    return positions, velocities, dim


def resolve_metric_params(
    recording: Recording, neighbor_radius: float | None, desired_distance: float | None
) -> tuple[float, float]:
    meta_params = recording.meta.algorithm_params or {}
    if neighbor_radius is None:
        neighbor_radius = meta_params.get("neighbor_radius")
    try:
        if neighbor_radius is not None and (not math.isfinite(float(neighbor_radius)) or float(neighbor_radius) <= 0):
            print(
                f"Предупреждение: neighbor_radius={neighbor_radius} в метаданных некорректен, игнорирую",
                file=sys.stderr,
            )
            neighbor_radius = None
    except Exception:
        neighbor_radius = None
    if neighbor_radius is None and recording.meta.algorithm_id == "flocking":
        neighbor_radius = FLOCK_PARAM_FALLBACKS.get("neighbor_radius")
        print(
            "Предупреждение: neighbor_radius не найден, "
            f"использую {neighbor_radius}",
            file=sys.stderr,
        )
    if neighbor_radius is None and recording.meta.algorithm_id == "flocking-alpha":
        neighbor_radius = FLOCK_ALPHA_PARAM_FALLBACKS.get("neighbor_radius")
        print(
            "Предупреждение: neighbor_radius не найден, "
            f"использую {neighbor_radius}",
            file=sys.stderr,
        )
    if neighbor_radius is None:
        neighbor_radius = FLOCK_ALPHA_PARAM_FALLBACKS.get("neighbor_radius")
        print(
            "Предупреждение: neighbor_radius не найден для алгоритма "
            f"'{recording.meta.algorithm_id}', использую {neighbor_radius}",
            file=sys.stderr,
        )
    if neighbor_radius is None or neighbor_radius <= 0:
        raise ProtoError("neighbor_radius не задан или <= 0")

    if desired_distance is None:
        desired_distance = meta_params.get("desired_distance")
    if desired_distance is None:
        desired_distance = meta_params.get("separation_radius")
    if desired_distance is None:
        desired_distance = meta_params.get("neighbor_radius")
    try:
        if desired_distance is not None and (not math.isfinite(float(desired_distance)) or float(desired_distance) <= 0):
            print(
                f"Предупреждение: desired_distance={desired_distance} в метаданных некорректен, игнорирую",
                file=sys.stderr,
            )
            desired_distance = None
    except Exception:
        desired_distance = None
    if desired_distance is None and recording.meta.algorithm_id == "flocking":
        desired_distance = FLOCK_PARAM_FALLBACKS.get("separation_radius")
        print(
            "Предупреждение: desired_distance не найден, "
            f"использую {desired_distance}",
            file=sys.stderr,
        )
    if desired_distance is None and recording.meta.algorithm_id == "flocking-alpha":
        desired_distance = FLOCK_ALPHA_PARAM_FALLBACKS.get("desired_distance")
        print(
            "Предупреждение: desired_distance не найден, "
            f"использую {desired_distance}",
            file=sys.stderr,
        )
    if desired_distance is None:
        desired_distance = FLOCK_ALPHA_PARAM_FALLBACKS.get("desired_distance")
        print(
            "Предупреждение: desired_distance не найден для алгоритма "
            f"'{recording.meta.algorithm_id}', использую {desired_distance}",
            file=sys.stderr,
        )
    if desired_distance is None or desired_distance <= 0:
        raise ProtoError("desired_distance не задан или <= 0")

    return float(neighbor_radius), float(desired_distance)


def compute_flock_metrics(
    recording: Recording, neighbor_radius: float, desired_distance: float
) -> dict[str, np.ndarray]:
    positions, velocities, _ = extract_state_vectors(recording)
    frame_count = recording.frame_count
    agent_count = recording.agent_count
    if agent_count <= 0:
        raise ProtoError("agent_count = 0")

    time = recording.time
    r2 = neighbor_radius * neighbor_radius
    d2 = desired_distance * desired_distance

    cohesion_radius = np.zeros(frame_count, dtype=np.float64)
    velocity_mismatch = np.zeros(frame_count, dtype=np.float64)
    deviation_energy = np.zeros(frame_count, dtype=np.float64)
    connectivity = np.zeros(frame_count, dtype=np.float64)

    for frame in range(frame_count):
        pos = positions[frame]
        vel = velocities[frame]
        center = pos.mean(axis=0)
        v_center = vel.mean(axis=0)

        offsets = pos - center
        cohesion_radius[frame] = np.linalg.norm(offsets, axis=1).max()

        v_offsets = vel - v_center
        velocity_mismatch[frame] = 0.5 * float(np.sum(v_offsets * v_offsets))

        parents = list(range(agent_count))
        ranks = [0] * agent_count

        def find_root(x: int) -> int:
            while parents[x] != x:
                parents[x] = parents[parents[x]]
                x = parents[x]
            return x

        def union(a: int, b: int) -> None:
            ra = find_root(a)
            rb = find_root(b)
            if ra == rb:
                return
            if ranks[ra] < ranks[rb]:
                parents[ra] = rb
            elif ranks[ra] > ranks[rb]:
                parents[rb] = ra
            else:
                parents[rb] = ra
                ranks[ra] += 1

        energy = 0.0
        for i in range(agent_count):
            pi = pos[i]
            for j in range(i + 1, agent_count):
                diff = pi - pos[j]
                dist2 = float(np.dot(diff, diff))
                if dist2 <= r2:
                    union(i, j)
                    dist = math.sqrt(dist2)
                    delta = dist - desired_distance
                    energy += delta * delta

        deviation_energy[frame] = energy
        roots = {find_root(i) for i in range(agent_count)}
        components = len(roots)
        if agent_count <= 1:
            connectivity[frame] = 1.0
        else:
            connectivity[frame] = (agent_count - components) / (agent_count - 1)

    velocity_mismatch /= agent_count
    if d2 > 0:
        deviation_energy /= d2
    else:
        deviation_energy[:] = math.nan

    return {
        "t": time,
        "C": connectivity,
        "R": cohesion_radius,
        "E": deviation_energy,
        "K": velocity_mismatch,
    }


def resolve_safety_params(
    recording: Recording, agent_safe_distance: float | None, scenario: str | None
) -> tuple[float, list[ObstaclePoly] | None]:
    meta_params = recording.meta.algorithm_params or {}
    d_safe = agent_safe_distance
    if d_safe is None:
        d_safe = meta_params.get("agent_safe_distance")
    if d_safe is None:
        d_safe = meta_params.get("separation_radius")
    if d_safe is None:
        d_safe = meta_params.get("desired_distance")
    if d_safe is None:
        d_safe = FLOCK_PARAM_FALLBACKS.get("separation_radius")
        print(
            f"Предупреждение: agent_safe_distance не найден, использую {d_safe}",
            file=sys.stderr,
        )
    try:
        d_safe = float(d_safe)
    except Exception as exc:
        raise ProtoError(f"agent_safe_distance не число ({exc})")
    if not math.isfinite(d_safe) or d_safe <= 0:
        raise ProtoError("agent_safe_distance не задан или <= 0")

    mode = (scenario or "auto").strip().lower()
    if mode not in ("auto", "paper", "none"):
        raise ProtoError("scenario должен быть auto|paper|none")
    obstacles = None
    if mode == "paper":
        obstacles = paper_obstacles()
    elif mode == "auto":
        if recording.meta.algorithm_id in ("formation-ecbf", "safe-flocking-alpha"):
            obstacles = paper_obstacles()
    return d_safe, obstacles


def compute_control_metrics(recording: Recording) -> dict[str, np.ndarray]:
    fields = recording.fields
    idx = {name: field_index(fields, name) for name in (
        "unx", "uny", "unz", "ux", "uy", "uz", "slack", "active", "constraints"
    )}
    missing = [k for k, v in idx.items() if v is None]
    if missing:
        return {}

    un = recording.states[:, :, [idx["unx"], idx["uny"], idx["unz"]]]
    u = recording.states[:, :, [idx["ux"], idx["uy"], idx["uz"]]]
    du = u - un
    du_norm = np.linalg.norm(du, axis=2)

    slack = recording.states[:, :, idx["slack"]]
    active = recording.states[:, :, idx["active"]]
    total = recording.states[:, :, idx["constraints"]]

    denom = np.sum(total, axis=1)
    active_frac = np.zeros(recording.frame_count, dtype=np.float64)
    mask = denom > 0
    active_frac[mask] = np.sum(active, axis=1)[mask] / denom[mask]

    return {
        "t": recording.time,
        "u_dev_mean": np.mean(du_norm, axis=1),
        "u_dev_max": np.max(du_norm, axis=1),
        "slack_mean": np.mean(slack, axis=1),
        "slack_max": np.max(slack, axis=1),
        "active_frac": active_frac,
    }


def compute_safety_metrics(
    recording: Recording, agent_safe_distance: float, obstacles: list[ObstaclePoly] | None
) -> dict[str, np.ndarray]:
    positions, _, dim = extract_state_vectors(recording)
    frame_count = recording.frame_count
    agent_count = recording.agent_count
    if agent_count <= 1:
        return {}

    d_safe = float(agent_safe_distance)
    d_safe2 = d_safe * d_safe

    min_pair_clearance = np.zeros(frame_count, dtype=np.float64)
    pair_violations = np.zeros(frame_count, dtype=np.float64)

    for frame in range(frame_count):
        pos = positions[frame]  # (N,dim)
        g = pos @ pos.T
        sq = np.sum(pos * pos, axis=1)
        d2 = sq[:, None] + sq[None, :] - 2.0 * g
        d2[d2 < 0] = 0.0
        np.fill_diagonal(d2, np.inf)
        min_d2 = float(np.min(d2))
        min_pair_clearance[frame] = math.sqrt(min_d2) - d_safe
        pair_violations[frame] = float(np.sum(np.triu(d2 < d_safe2, 1)))

    out: dict[str, np.ndarray] = {
        "t": recording.time,
        "min_pair_clearance": min_pair_clearance,
        "pair_violations": pair_violations,
    }

    if obstacles:
        time = recording.time
        min_ob_clearance = np.full(frame_count, math.inf, dtype=np.float64)
        ob_violations = np.zeros(frame_count, dtype=np.float64)

        for ob in obstacles:
            p_ob = ob.pos(time)[:, :dim]
            diff = positions - p_ob[:, None, :]
            dist = np.linalg.norm(diff, axis=2)
            clearance = dist - float(ob.d)
            min_ob_clearance = np.minimum(min_ob_clearance, np.min(clearance, axis=1))
            ob_violations += np.sum(clearance < 0.0, axis=1).astype(np.float64)

        out["min_obstacle_clearance"] = min_ob_clearance
        out["obstacle_violations"] = ob_violations

    return out


def print_info(path: Path, recording: Recording) -> None:
    meta = recording.meta
    print(f"Файл: {path}")
    print(f"Кадров: {recording.frame_count}")
    print(f"Агентов: {recording.agent_count}")
    print(f"Поля: {', '.join(recording.fields)}")
    print(f"dt: {meta.dt} с")
    print(f"stride: {meta.stride}")
    if meta.created_at:
        print(f"Создано: {meta.created_at}")
    if meta.model_id:
        print(f"Модель: {meta.model_id}")
    if meta.algorithm_id:
        print(f"Алгоритм: {meta.algorithm_id}")
    if meta.algorithm_params:
        params = ", ".join(
            f"{key}={value:.6g}" for key, value in sorted(meta.algorithm_params.items())
        )
        print(f"Параметры алгоритма: {params}")
    print(f"Плоскость 2D: {'да' if meta.plane2d else 'нет'}")


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Чтение rphys protobuf траекторий и построение графиков."
    )
    parser.add_argument("file", type=Path, help="Путь к файлу .pb")
    parser.add_argument("--x", default="t", help="Ось X (t, x, y, z, vx, vy, vz)")
    parser.add_argument("--y", default="x", help="Ось Y (t, x, y, z, vx, vy, vz)")
    parser.add_argument(
        "--agent", type=int, default=0, help="Индекс агента (с 0)"
    )
    parser.add_argument(
        "--all", action="store_true", help="Построить график для всех агентов"
    )
    parser.add_argument("--out", type=Path, help="Сохранить график в файл")
    parser.add_argument("--dpi", type=int, default=150, help="DPI для сохранения")
    parser.add_argument("--title", default="", help="Заголовок графика")
    parser.add_argument(
        "--backend",
        default="auto",
        help="Backend Matplotlib (auto, QtAgg, Agg)",
    )
    parser.add_argument(
        "--metrics",
        action="store_true",
        help="Построить метрики флокинга: C(t), R(t), E~(t), K~(t)",
    )
    parser.add_argument(
        "--split-metrics",
        action="store_true",
        help="Сохранить каждую метрику отдельным графиком (требует --metrics)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Папка для сохранения графиков (по умолчанию рядом с .pb)",
    )
    parser.add_argument(
        "--prefix",
        default=None,
        help="Префикс имён файлов при --split-metrics (по умолчанию stem файла .pb)",
    )
    parser.add_argument(
        "--neighbor-radius",
        type=float,
        default=None,
        help="Радиус соседства r, м (для C(t) и E~(t))",
    )
    parser.add_argument(
        "--desired-distance",
        type=float,
        default=None,
        help="Желаемая дистанция d, м (для E~(t))",
    )
    parser.add_argument(
        "--agent-safe-distance",
        type=float,
        default=None,
        help="Безопасная дистанция между агентами d_safe, м (для safety-метрик)",
    )
    parser.add_argument(
        "--scenario",
        default="auto",
        help="Сценарий препятствий для safety-метрик: auto|paper|none",
    )
    parser.add_argument(
        "--info", action="store_true", help="Показать метаданные и выйти"
    )
    parser.add_argument(
        "--list-axes", action="store_true", help="Показать доступные оси и выйти"
    )
    return parser.parse_args(argv)


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def save_metric_plot(
    *,
    out_path: Path,
    t: np.ndarray,
    y: np.ndarray,
    title: str,
    ylabel: str,
    zero_line: bool,
    dpi: int,
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5.4))
    ax.plot(t, y, linewidth=1.6)
    ax.set_title(title)
    ax.set_xlabel("время, с")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    if zero_line:
        ax.axhline(0.0, color="#ff6b6b", linewidth=1.2, alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def save_split_metric_plots(
    *,
    file_path: Path,
    out_dir: Path | None,
    prefix: str | None,
    dpi: int,
    plots: list[tuple[str, str, str, np.ndarray, bool]],
    t: np.ndarray,
) -> Path:
    # Default next to input .pb so results stay with the experiment.
    target_dir = out_dir or (file_path.parent / f"{file_path.stem}-plots")
    ensure_dir(target_dir)
    name_prefix = prefix or file_path.stem

    for key, title, ylabel, series, zero_line in plots:
        safe_key = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in key)
        out_path = target_dir / f"{name_prefix}-{safe_key}.png"
        save_metric_plot(
            out_path=out_path,
            t=t,
            y=series,
            title=title,
            ylabel=ylabel,
            zero_line=zero_line,
            dpi=dpi,
        )

    return target_dir


def configure_matplotlib(out_path: Path | None) -> None:
    if out_path:
        import matplotlib

        matplotlib.use("Agg")

def normalize_backend_name(name: str | None) -> str | None:
    if not name:
        return None
    raw = name.strip().lower()
    if raw in ("auto",):
        return None
    if raw in ("qt", "qtagg", "qt5agg", "qt6agg"):
        return "QtAgg"
    if raw == "agg":
        return "Agg"
    return name


def try_set_backend(backend: str, verbose: bool) -> bool:
    try:
        import matplotlib
    except Exception as exc:
        if verbose:
            print(f"Предупреждение: matplotlib недоступен ({exc})", file=sys.stderr)
        return False
    try:
        matplotlib.use(backend, force=True)
        return True
    except Exception as exc:
        if verbose:
            print(
                f"Предупреждение: не удалось включить backend {backend}: {exc}",
                file=sys.stderr,
            )
        return False


def is_non_interactive_backend() -> bool:
    try:
        import matplotlib
    except Exception:
        return False
    backend_name = matplotlib.get_backend().lower()
    try:
        from matplotlib.backends import backend_registry, BackendFilter

        non_interactive = backend_registry.list_builtin(
            BackendFilter.NON_INTERACTIVE
        )
        return backend_name in {name.lower() for name in non_interactive}
    except Exception:
        try:
            from matplotlib import rcsetup

            return backend_name in {name.lower() for name in rcsetup.non_interactive_bk}
        except Exception:
            return False


def is_headless() -> bool:
    if sys.platform.startswith("linux"):
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
            return False
        return True
    return False


def pick_default_out(base_name: str = "rphys-plot.png") -> Path:
    base = Path.cwd() / base_name
    if not base.exists():
        return base
    stem = base.stem
    suffix = base.suffix or ".png"
    for idx in range(1, 1000):
        candidate = Path.cwd() / f"{stem}-{idx}{suffix}"
        if not candidate.exists():
            return candidate
    raise ProtoError("unable to find a free filename for output plot")


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    neighbor_radius = None
    desired_distance = None
    try:
        recording = build_recording(args.file)
        if args.info:
            print_info(args.file, recording)
            if not args.list_axes and not args.metrics:
                return
        if args.list_axes:
            axes = list_axes(recording.fields)
            print("Доступные оси:")
            for axis in axes:
                print(f"- {axis}: {axis_label(axis)}")
            return

        if args.metrics:
            neighbor_radius, desired_distance = resolve_metric_params(
                recording, args.neighbor_radius, args.desired_distance
            )
        else:
            x_axis = normalize_axis(args.x)
            y_axis = normalize_axis(args.y)
            axis_label(x_axis)
            axis_label(y_axis)

            if not args.all and (args.agent < 0 or args.agent >= recording.agent_count):
                raise ProtoError(
                    f"agent index {args.agent} out of range (0..{recording.agent_count - 1})"
                )
    except (ProtoError, OSError) as exc:
        print(f"Ошибка: {exc}", file=sys.stderr)
        sys.exit(1)

    headless = is_headless()
    out_path = args.out
    if out_path is None and headless:
        out_path = pick_default_out()
        print(
            "Предупреждение: интерактивное окно недоступно, "
            f"сохраняю график в файл {out_path}",
            file=sys.stderr,
        )

    backend_choice = normalize_backend_name(args.backend)
    if backend_choice:
        if not try_set_backend(backend_choice, verbose=True) and out_path is None:
            out_path = pick_default_out()
            print(
                "Предупреждение: интерактивное окно недоступно, "
                f"сохраняю график в файл {out_path}",
                file=sys.stderr,
            )
            try_set_backend("Agg", verbose=False)
    elif out_path is not None:
        try_set_backend("Agg", verbose=False)
    else:
        try_set_backend(PREFERRED_INTERACTIVE_BACKEND, verbose=False)

    if out_path is None and (is_non_interactive_backend() or headless):
        out_path = pick_default_out()
        print(
            "Предупреждение: интерактивное окно недоступно, "
            f"сохраняю график в файл {out_path}",
            file=sys.stderr,
        )
        try_set_backend("Agg", verbose=False)

    import matplotlib.pyplot as plt

    if args.metrics:
        metrics = compute_flock_metrics(
            recording,
            neighbor_radius if neighbor_radius is not None else 0.0,
            desired_distance if desired_distance is not None else 0.0,
        )
        d_safe, obstacles = resolve_safety_params(
            recording, args.agent_safe_distance, args.scenario
        )
        safety = compute_safety_metrics(recording, d_safe, obstacles)
        control = compute_control_metrics(recording)

        plots: list[tuple[str, str, str, np.ndarray, bool]] = []
        plots.append(("C", "Связность", "C(t), доля", metrics["C"], False))
        plots.append(("R", "Радиус когезии", "R(t), м", metrics["R"], False))
        plots.append(("E", "Отклонение решетки", "E~(t), безразм.", metrics["E"], False))
        plots.append(("K", "Несогласованность скоростей", "K~(t), м^2/с^2", metrics["K"], False))

        if safety:
            plots.append(
                ("d_agents", "Мин. клиренс агент-агент", "min(d_ij - d_safe), м", safety["min_pair_clearance"], True)
            )
            plots.append(
                ("viol_agents", "Нарушения агент-агент", "пар (d_ij < d_safe)", safety["pair_violations"], False)
            )
            if "min_obstacle_clearance" in safety:
                plots.append(
                    ("d_obs", "Мин. клиренс до препятствий", "min(d_i - d_ob), м", safety["min_obstacle_clearance"], True)
                )
                plots.append(
                    ("viol_obs", "Нарушения препятствий", "пар (d_i < d_ob)", safety["obstacle_violations"], False)
                )

        if control:
            plots.append(("u_dev", "Отклонение управления", "mean ||u*-u_n||", control["u_dev_mean"], False))
            plots.append(("slack", "Slack", "mean slack", control["slack_mean"], False))
            plots.append(("active", "Активные ограничения", "sum(active)/sum(total)", control["active_frac"], False))

        if args.split_metrics:
            # Always save; avoid opening interactive windows.
            out_dir = save_split_metric_plots(
                file_path=args.file,
                out_dir=args.out_dir,
                prefix=args.prefix,
                dpi=args.dpi,
                plots=plots,
                t=metrics["t"],
            )
            print(f"Сохранено: {out_dir}")
            return

        cols = 2 if len(plots) <= 6 else 3
        rows = int(math.ceil(len(plots) / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 3.0 * rows), sharex=True)
        axes = np.array(axes).ravel()

        t = metrics["t"]
        for idx, (_, title, ylabel, series, zero_line) in enumerate(plots):
            ax = axes[idx]
            ax.plot(t, series, linewidth=1.4)
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            if zero_line:
                ax.axhline(0.0, color="#ff6b6b", linewidth=1.0, alpha=0.45)
            ax.set_xlabel("время, с")
            ax.grid(True, alpha=0.3)

        for ax in axes[len(plots):]:
            ax.axis("off")

        if args.title:
            fig.suptitle(args.title)
            fig.tight_layout(rect=[0, 0, 1, 0.96])
        else:
            fig.tight_layout()
    else:
        fig, ax = plt.subplots(figsize=(9, 5.4))
        if args.all:
            for agent in range(recording.agent_count):
                x_data = axis_series(recording, x_axis, agent)
                y_data = axis_series(recording, y_axis, agent)
                ax.plot(x_data, y_data, alpha=0.35, linewidth=1)
        else:
            x_data = axis_series(recording, x_axis, args.agent)
            y_data = axis_series(recording, y_axis, args.agent)
            ax.plot(x_data, y_data, linewidth=1.6, label=f"агент {args.agent}")
            ax.legend(loc="best")

        ax.set_xlabel(axis_label(x_axis))
        ax.set_ylabel(axis_label(y_axis))
        if args.title:
            ax.set_title(args.title)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=args.dpi)
        print(f"Сохранено: {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
