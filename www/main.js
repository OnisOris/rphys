import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import init, {
  WasmSim,
  available_models,
  available_algorithms,
  algorithms_for_model,
} from "./pkg/rphys.js";

await init();

const viewport = document.getElementById("viewport");
const homeButton = document.getElementById("home-btn");
const pauseButton = document.getElementById("sim-toggle-btn");
const restartButton = document.getElementById("sim-restart-btn");
const modelSelect = document.getElementById("model-select");
const algorithmSelect = document.getElementById("algorithm-select");
const setupToggleBtn = document.getElementById("setup-toggle-btn");
const setupPanel = document.getElementById("setup-panel");
const applySetupBtn = document.getElementById("apply-setup-btn");
const closeSetupBtn = document.getElementById("close-setup-btn");
const dtInput = document.getElementById("dt-input");
const clusterList = document.getElementById("cluster-list");
const addClusterBtn = document.getElementById("add-cluster-btn");
const agentTotalEl = document.getElementById("agent-total");
const algorithmSelectPanel = document.getElementById("algorithm-select-panel");
const panelPlane2dToggle = document.getElementById("panel-plane2d-toggle");
const groupColorList = document.getElementById("group-color-list");
const saveConfigBtn = document.getElementById("save-config-btn");
const loadConfigBtn = document.getElementById("load-config-btn");
const configFileInput = document.getElementById("config-file-input");

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.setSize(viewport.clientWidth, viewport.clientHeight, false);
viewport.appendChild(renderer.domElement);

const gizmoContainer = document.createElement("div");
gizmoContainer.style.position = "absolute";
gizmoContainer.style.top = "10px";
gizmoContainer.style.right = "14px";
gizmoContainer.style.pointerEvents = "none";
viewport.appendChild(gizmoContainer);

const gizmoRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
gizmoRenderer.setPixelRatio(window.devicePixelRatio || 1);
const gizmoDisplaySize = 180;
const gizmoBufferSize = 420; // extra space so labels/axes are not clipped
gizmoRenderer.setSize(gizmoBufferSize, gizmoBufferSize, false);
gizmoRenderer.domElement.style.width = `${gizmoDisplaySize}px`;
gizmoRenderer.domElement.style.height = `${gizmoDisplaySize}px`;
gizmoRenderer.domElement.style.display = "block";
gizmoRenderer.domElement.style.pointerEvents = "none";
gizmoContainer.appendChild(gizmoRenderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);

const camera = new THREE.PerspectiveCamera(
  45,
  viewport.clientWidth / viewport.clientHeight,
  0.1,
  100
);
camera.position.set(0, 0, 12);

const orbitTarget = new THREE.Vector3(0, 0, 0);
const orbit = new THREE.Spherical().setFromVector3(camera.position);
const orbitRotateSpeed = 0.004;
const orbitZoomSpeed = 0.0025;
const orbitMinPhi = 0.15;
const orbitMaxPhi = Math.PI - 0.15;
const orbitMinRadius = 4.0;
const orbitMaxRadius = 40.0;
const homeOrbit = orbit.clone();
const homeTarget = orbitTarget.clone();
const homeUp = new THREE.Vector3(0, 1, 0);
const orbitOffset = new THREE.Vector3();
const panLast = new THREE.Vector2();
const panDir = new THREE.Vector3();
const panRight = new THREE.Vector3();
const panUp = new THREE.Vector3();
const panScale = 0.0025;

function updateCameraFromOrbit() {
  orbitOffset.setFromSpherical(orbit);
  camera.position.copy(orbitTarget).add(orbitOffset);
  camera.lookAt(orbitTarget);
}

updateCameraFromOrbit();

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function resetCamera() {
  orbit.copy(homeOrbit);
  orbitTarget.copy(homeTarget);
  camera.up.copy(homeUp);
  updateCameraFromOrbit();
}

if (homeButton) {
  homeButton.addEventListener("click", resetCamera);
}

const ambient = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambient);
const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
keyLight.position.set(6, 10, 8);
scene.add(keyLight);

const helpersGroup = new THREE.Group();
scene.add(helpersGroup);

function tuneGrid(grid, opacity) {
  const mats = Array.isArray(grid.material) ? grid.material : [grid.material];
  mats.forEach((mat) => {
    mat.transparent = true;
    mat.opacity = opacity;
    mat.depthWrite = false;
  });
  return grid;
}

const gridSize = 20;
const gridDivisions = 20;
const gridXY = tuneGrid(new THREE.GridHelper(gridSize, gridDivisions, 0x5aa6ff, 0x1f3f7a), 0.35);
gridXY.rotation.x = Math.PI / 2;
const gridXZ = tuneGrid(new THREE.GridHelper(gridSize, gridDivisions, 0x58d07a, 0x1f6b39), 0.35);
const gridYZ = tuneGrid(new THREE.GridHelper(gridSize, gridDivisions, 0xff5a5a, 0x7a1f1f), 0.35);
gridYZ.rotation.z = Math.PI / 2;

helpersGroup.add(gridXY, gridXZ, gridYZ);

const axisLength = 4.0;
const axisHeadLength = 0.45;
const axisHeadWidth = 0.25;
const axisX = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0, 0),
  axisLength,
  0xff5a5a,
  axisHeadLength,
  axisHeadWidth
);
const axisY = new THREE.ArrowHelper(
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(0, 0, 0),
  axisLength,
  0x58d07a,
  axisHeadLength,
  axisHeadWidth
);
const axisZ = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1),
  new THREE.Vector3(0, 0, 0),
  axisLength,
  0x5aa6ff,
  axisHeadLength,
  axisHeadWidth
);

helpersGroup.add(axisX, axisY, axisZ);

function bindToggle(id, target, fallback) {
  const el = document.getElementById(id);
  if (!el) {
    target.visible = fallback;
    return;
  }
  target.visible = el.checked;
  el.addEventListener("change", () => {
    target.visible = el.checked;
  });
}

bindToggle("toggle-plane-xy", gridXY, false);
bindToggle("toggle-plane-xz", gridXZ, true);
bindToggle("toggle-plane-yz", gridYZ, false);
bindToggle("toggle-axis-x", axisX, true);
bindToggle("toggle-axis-y", axisY, true);
bindToggle("toggle-axis-z", axisZ, true);

const plane2dToggle = document.getElementById("toggle-plane-2d");

let modelCatalog = Array.from(available_models());
let algorithmCatalog = Array.from(available_algorithms());

const customModelId = "from-states";

let clusters = [
  {
    name: "Sphere A",
    count: 24,
    radius: 5,
    center: { x: 0, y: 0, z: 0 },
    velocity: { x: 0, y: 0, z: 0 },
    radialSpeed: 0,
    drag: 0.08,
    group: 0,
  },
  {
    name: "Sphere B",
    count: 16,
    radius: 4,
    center: { x: 6, y: 0, z: 0 },
    velocity: { x: 0, y: 0, z: 0 },
    radialSpeed: 0,
    drag: 0.08,
    group: 1,
  },
];
let customDt = 1 / 60;
let activeModelId = null;
let activeAlgorithmId = null;
let currentGroups = [];

const groupPalette = [
  "#6ad1ff",
  "#58d07a",
  "#ff5a5a",
  "#f2c94c",
  "#9b51e0",
  "#f2994a",
  "#2d9cdb",
  "#eb5757",
];
const groupColors = new Map();

function colorForGroup(groupId) {
  const key = String(groupId);
  if (!groupColors.has(key)) {
    const idx = Math.abs(Number(groupId) || 0) % groupPalette.length;
    groupColors.set(key, groupPalette[idx]);
  }
  return groupColors.get(key);
}

function populateSelect(select, items, getValue, getLabel) {
  if (!select) return;
  select.innerHTML = "";
  items.forEach((item, index) => {
    const option = document.createElement("option");
    option.value = getValue(item);
    option.textContent = getLabel(item);
    option.title = item.description || "";
    if (index === 0) option.selected = true;
    select.appendChild(option);
  });
}

function currentModelId() {
  if (modelSelect && modelSelect.value) return modelSelect.value;
  if (modelCatalog.length > 0) return modelCatalog[0].id;
  return "ring-swarm";
}

function currentAlgorithmId(modelId) {
  if (algorithmSelect && algorithmSelect.value) return algorithmSelect.value;
  const model = modelCatalog.find((m) => m.id === modelId);
  if (model && model.defaultAlgorithm) return model.defaultAlgorithm;
  if (algorithmCatalog.length > 0) return algorithmCatalog[0].id;
  return "flocking";
}

function refreshAlgorithmSelect(modelId, preferredId) {
  const compatible = Array.from(algorithms_for_model(modelId));
  if (algorithmSelect) {
    populateSelect(algorithmSelect, compatible, (a) => a.id, (a) => a.name);
  }
  if (algorithmSelectPanel) {
    populateSelect(algorithmSelectPanel, compatible, (a) => a.id, (a) => a.name);
  }
  const preferred = preferredId || currentAlgorithmId(modelId);
  if (compatible.some((a) => a.id === preferred)) {
    if (algorithmSelect) algorithmSelect.value = preferred;
    if (algorithmSelectPanel) algorithmSelectPanel.value = preferred;
  }
}

if (modelSelect) {
  populateSelect(modelSelect, modelCatalog, (m) => m.id, (m) => m.name);
  modelSelect.addEventListener("change", () => {
    refreshAlgorithmSelect(modelSelect.value);
    resetSimulation();
  });
}

if (algorithmSelect) {
  refreshAlgorithmSelect(currentModelId());
  algorithmSelect.addEventListener("change", () => {
    if (algorithmSelectPanel) algorithmSelectPanel.value = algorithmSelect.value;
    const targetModel = currentModelId();
    const targetAlgo = algorithmSelect.value;
    if (activeModelId && targetModel === activeModelId && sim) {
      try {
        sim.set_algorithm(targetAlgo);
        activeAlgorithmId = targetAlgo;
        return;
      } catch (err) {
        console.error("set_algorithm failed, recreating sim", err);
      }
    }
    resetSimulation();
  });
}

if (algorithmSelectPanel) {
  populateSelect(algorithmSelectPanel, algorithmCatalog, (a) => a.id, (a) => a.name);
  algorithmSelectPanel.addEventListener("change", () => {
    if (algorithmSelect) {
      algorithmSelect.value = algorithmSelectPanel.value;
    }
  });
}

function syncAlgorithmSelects(newId) {
  if (algorithmSelect) algorithmSelect.value = newId;
  if (algorithmSelectPanel) algorithmSelectPanel.value = newId;
}

const gizmoScene = new THREE.Scene();
gizmoScene.background = null;
const gizmoCamera = new THREE.PerspectiveCamera(35, 1, 0.1, 10);
gizmoCamera.position.set(0, 0, 4.2);
gizmoCamera.lookAt(0, 0, 0);

const gizmoCubeGeo = new THREE.BoxGeometry(1, 1, 1);
const gizmoCube = new THREE.Mesh(gizmoCubeGeo, [
  new THREE.MeshBasicMaterial({ color: 0xff5a5a }), // +X
  new THREE.MeshBasicMaterial({ color: 0x7a1f1f }), // -X
  new THREE.MeshBasicMaterial({ color: 0x58d07a }), // +Y
  new THREE.MeshBasicMaterial({ color: 0x1f6b39 }), // -Y
  new THREE.MeshBasicMaterial({ color: 0x5aa6ff }), // +Z
  new THREE.MeshBasicMaterial({ color: 0x1f3f7a }), // -Z
]);

const gizmoEdges = new THREE.LineSegments(
  new THREE.EdgesGeometry(gizmoCubeGeo),
  new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.6 })
);
gizmoEdges.scale.setScalar(1.02);

const gizmoAxes = new THREE.AxesHelper(1.6);
const gizmoRoot = new THREE.Group();
gizmoRoot.scale.setScalar(0.75);
gizmoRoot.add(gizmoCube, gizmoEdges, gizmoAxes);
gizmoScene.add(gizmoRoot);

const gizmoViewportSize = new THREE.Vector2();
const gizmoSize = gizmoDisplaySize;
const gizmoMargin = 0;
const gizmoPointer = new THREE.Vector2();
const gizmoRaycaster = new THREE.Raycaster();
const gizmoAxis = new THREE.Vector3();
const gizmoOffset = new THREE.Vector3();

function makeAxisLabel(text, color) {
  const canvas = document.createElement("canvas");
  const size = 128;
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, size, size);
  ctx.fillStyle = "rgba(10, 14, 20, 0.75)";
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size * 0.38, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
  ctx.lineWidth = 4;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.font = "700 64px 'JetBrains Mono', monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, size / 2, size / 2 + 2);

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  const material = new THREE.SpriteMaterial({
    map: texture,
    depthTest: false,
    depthWrite: false,
    transparent: true,
  });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(0.45, 0.45, 0.45);
  return sprite;
}

const labelX = makeAxisLabel("X", "#ff5a5a");
const labelY = makeAxisLabel("Y", "#58d07a");
const labelZ = makeAxisLabel("Z", "#5aa6ff");
labelX.position.set(0.95, 0.0, 0.0);
labelY.position.set(0.0, 0.95, 0.0);
labelZ.position.set(0.0, 0.0, 0.95);
gizmoRoot.add(labelX, labelY, labelZ);

function getGizmoPointer(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  if (rect.width === 0 || rect.height === 0) return null;
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const left = rect.width - gizmoSize - gizmoMargin;
  const top = gizmoMargin;
  if (x < left || x > left + gizmoSize || y < top || y > top + gizmoSize) {
    return null;
  }
  const localX = x - left;
  const localY = y - top;
  gizmoPointer.set(
    (localX / gizmoSize) * 2 - 1,
    -(localY / gizmoSize) * 2 + 1
  );
  return gizmoPointer;
}

function axisFromNormal(normal) {
  const ax = Math.abs(normal.x);
  const ay = Math.abs(normal.y);
  const az = Math.abs(normal.z);
  if (ax >= ay && ax >= az) {
    gizmoAxis.set(Math.sign(normal.x) || 1, 0, 0);
  } else if (ay >= ax && ay >= az) {
    gizmoAxis.set(0, Math.sign(normal.y) || 1, 0);
  } else {
    gizmoAxis.set(0, 0, Math.sign(normal.z) || 1);
  }
  return gizmoAxis;
}

function alignCameraToAxis(axis) {
  if (Math.abs(axis.y) > 0.5) {
    camera.up.set(0, 0, axis.y > 0 ? 1 : -1);
  } else {
    camera.up.copy(homeUp);
  }
  gizmoOffset.copy(axis).normalize().multiplyScalar(orbit.radius);
  orbit.setFromVector3(gizmoOffset);
  updateCameraFromOrbit();
}

function onGizmoDoubleClick(event) {
  if (event.button !== 0) return;
  if (!isInViewport(event)) return;
  const pointer = getGizmoPointer(event);
  if (!pointer) return;
  event.preventDefault();
  gizmoRaycaster.setFromCamera(pointer, gizmoCamera);
  const hits = gizmoRaycaster.intersectObject(gizmoCube, false);
  if (hits.length === 0 || !hits[0].face) return;
  const axis = axisFromNormal(hits[0].face.normal);
  alignCameraToAxis(axis);
}

let sim = null;
let paused = false;

function setPaused(next) {
  paused = next;
  if (!pauseButton) return;
  pauseButton.dataset.state = paused ? "paused" : "running";
  pauseButton.textContent = paused ? "Resume" : "Pause";
  pauseButton.setAttribute("aria-pressed", paused ? "true" : "false");
}

const geometry = new THREE.SphereGeometry(0.12, 16, 16);
const baseMaterial = new THREE.MeshStandardMaterial({
  color: 0x6ad1ff,
  roughness: 0.35,
  metalness: 0.1,
});

const meshes = [];

function refreshGroups() {
  currentGroups = [];
  if (!sim) return;
  if (typeof sim.groups === "function") {
    currentGroups = Array.from(sim.groups());
  }
  if (currentGroups.length === 0) {
    const count = sim.len();
    currentGroups = new Array(count).fill(0);
  }
}

function applyGroupColorsToMeshes() {
  const count = meshes.length;
  for (let i = 0; i < count; i += 1) {
    const groupId = currentGroups[i] ?? 0;
    const color = colorForGroup(groupId);
    meshes[i].material.color.set(color);
  }
}

function rebuildMeshes() {
  meshes.forEach((mesh) => scene.remove(mesh));
  meshes.length = 0;
  const count = sim ? sim.len() : 0;
  refreshGroups();
  for (let i = 0; i < count; i += 1) {
    const groupId = currentGroups[i] ?? 0;
    const material = baseMaterial.clone();
    material.color.set(colorForGroup(groupId));
    const mesh = new THREE.Mesh(geometry, material);
    mesh.userData.index = i;
    scene.add(mesh);
    meshes.push(mesh);
  }
}

function clusterCountTotal() {
  return clusters.reduce((sum, c) => sum + Number(c.count || 0), 0);
}

function updateAgentTotal() {
  if (!agentTotalEl) return;
  agentTotalEl.textContent = `${clusterCountTotal()} agents`;
}

function renderClusters() {
  if (!clusterList) return;
  clusterList.innerHTML = "";
  clusters.forEach((c, idx) => {
    const card = document.createElement("div");
    card.className = "cluster-card";
    card.dataset.index = idx;

    const header = document.createElement("div");
    header.className = "cluster-header";
    const title = document.createElement("div");
    title.className = "cluster-title";
    title.textContent = c.name || `Cluster ${idx + 1}`;
    const chip = document.createElement("span");
    chip.className = "chip";
    chip.textContent = `Sphere • ${c.count} agents`;
    const remove = document.createElement("button");
    remove.className = "icon-btn";
    remove.type = "button";
    remove.textContent = "✕";
    remove.addEventListener("click", () => {
      clusters.splice(idx, 1);
      renderClusters();
    });
    header.append(title, chip, remove);

    const row1 = document.createElement("div");
    row1.className = "field-row";
    row1.append(
      makeNumberField("Count", c.count, (v) => { c.count = v; chip.textContent = `Sphere • ${c.count} agents`; updateAgentTotal(); }),
      makeNumberField("Radius", c.radius, (v) => { c.radius = v; })
    );

    const row2 = document.createElement("div");
    row2.className = "field-row stack";
    row2.append(
      makeVectorField("Center", c.center, (vec) => { c.center = vec; }),
      makeVectorField("Velocity", c.velocity, (vec) => { c.velocity = vec; })
    );

    const row3 = document.createElement("div");
    row3.className = "field-row";
    row3.append(
      makeNumberField("Radial speed", c.radialSpeed, (v) => { c.radialSpeed = v; }),
      makeNumberField("Drag", c.drag, (v) => { c.drag = v; })
    );

    const row4 = document.createElement("div");
    row4.className = "field-row";
    row4.append(
      makeNumberField("Group", c.group, (v) => { c.group = Math.max(0, Math.round(v)); renderGroupColors(); }),
      makeTextField("Name", c.name, (v) => { c.name = v; title.textContent = v || `Cluster ${idx + 1}`; })
    );

    card.append(header, row1, row2, row3, row4);
    clusterList.append(card);
  });
  updateAgentTotal();
  renderGroupColors();
}

function makeNumberField(label, value, onChange) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";
  const lab = document.createElement("label");
  lab.textContent = label;
  const input = document.createElement("input");
  input.type = "number";
  input.value = value ?? 0;
  input.addEventListener("input", () => onChange(Number(input.value)));
  wrapper.append(lab, input);
  return wrapper;
}

function makeTextField(label, value, onChange) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";
  const lab = document.createElement("label");
  lab.textContent = label;
  const input = document.createElement("input");
  input.type = "text";
  input.value = value ?? "";
  input.addEventListener("input", () => onChange(input.value));
  wrapper.append(lab, input);
  return wrapper;
}

function makeVectorField(label, vec, onChange) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";

  const lab = document.createElement("label");
  lab.textContent = label + " (x, y, z)";
  wrapper.append(lab);

  const rowPos = document.createElement("div");
  rowPos.style.display = "grid";
  rowPos.style.gridTemplateColumns = "repeat(3, minmax(160px, 1fr))";
  rowPos.style.gap = "16px";

  const inputsPos = ["x", "y", "z"].map((axis) => {
    const input = document.createElement("input");
    input.type = "number";
    input.value = Number(vec?.[axis] ?? 0);
    input.placeholder = axis;
    input.addEventListener("input", () => {
      const next = {
        x: Number(inputsPos[0].value),
        y: Number(inputsPos[1].value),
        z: Number(inputsPos[2].value),
      };
      onChange(next);
    });
    return input;
  });
  inputsPos.forEach((i) => rowPos.append(i));

  wrapper.append(rowPos);
  return wrapper;
}

function collectGroupIds() {
  const ids = new Set();
  clusters.forEach((c) => ids.add(Number(c.group || 0)));
  if (ids.size === 0 && currentGroups.length > 0) {
    currentGroups.forEach((g) => ids.add(Number(g || 0)));
  }
  if (ids.size === 0) {
    ids.add(0);
  }
  return Array.from(ids).sort((a, b) => a - b);
}

function renderGroupColors() {
  if (!groupColorList) return;
  groupColorList.innerHTML = "";
  const ids = collectGroupIds();
  ids.forEach((id) => {
    const row = document.createElement("div");
    row.className = "group-color-row";
    const label = document.createElement("label");
    label.textContent = `Group ${id}`;
    const input = document.createElement("input");
    input.type = "color";
    input.value = colorForGroup(id);
    input.addEventListener("input", () => {
      groupColors.set(String(id), input.value);
      applyGroupColorsToMeshes();
    });
    row.append(label, input);
    groupColorList.append(row);
  });
}

function buildSetupConfig() {
  const algo = algorithmSelectPanel?.value || algorithmSelect?.value || currentAlgorithmId(customModelId);
  const plane2d = panelPlane2dToggle?.checked ?? plane2dToggle?.checked ?? false;
  const groupColorObj = {};
  groupColors.forEach((value, key) => {
    groupColorObj[key] = value;
  });
  return {
    version: 1,
    dt: Number(dtInput?.value) || customDt,
    algorithm: algo,
    plane2d,
    clusters: clusters.map((c) => ({
      name: c.name || "",
      count: Number(c.count || 0),
      radius: Number(c.radius || 0),
      center: [Number(c.center.x || 0), Number(c.center.y || 0), Number(c.center.z || 0)],
      velocity: [Number(c.velocity.x || 0), Number(c.velocity.y || 0), Number(c.velocity.z || 0)],
      radialSpeed: Number(c.radialSpeed || 0),
      drag: Number(c.drag || 0),
      group: Number(c.group || 0),
    })),
    groupColors: groupColorObj,
  };
}

function normalizeCluster(raw, fallbackIndex) {
  return {
    name: raw?.name || `Sphere ${fallbackIndex + 1}`,
    count: Number(raw?.count || 0),
    radius: Number(raw?.radius || 0),
    center: {
      x: Number(raw?.center?.[0] ?? raw?.center?.x ?? 0),
      y: Number(raw?.center?.[1] ?? raw?.center?.y ?? 0),
      z: Number(raw?.center?.[2] ?? raw?.center?.z ?? 0),
    },
    velocity: {
      x: Number(raw?.velocity?.[0] ?? raw?.velocity?.x ?? 0),
      y: Number(raw?.velocity?.[1] ?? raw?.velocity?.y ?? 0),
      z: Number(raw?.velocity?.[2] ?? raw?.velocity?.z ?? 0),
    },
    radialSpeed: Number(raw?.radialSpeed || 0),
    drag: Number(raw?.drag || 0),
    group: Math.max(0, Math.round(Number(raw?.group || 0))),
  };
}

function applySetupConfig(cfg, runNow) {
  if (!cfg || typeof cfg !== "object") return;

  if (typeof cfg.dt === "number" && !Number.isNaN(cfg.dt)) {
    customDt = cfg.dt;
    if (dtInput) dtInput.value = customDt.toFixed(3);
  }

  if (cfg.groupColors && typeof cfg.groupColors === "object") {
    groupColors.clear();
    Object.entries(cfg.groupColors).forEach(([key, value]) => {
      if (typeof value === "string") {
        groupColors.set(String(key), value);
      }
    });
  }

  const nextClusters = Array.isArray(cfg.clusters)
    ? cfg.clusters.map((c, idx) => normalizeCluster(c, idx))
    : [];
  clusters = nextClusters;

  const algo = typeof cfg.algorithm === "string" ? cfg.algorithm : currentAlgorithmId(customModelId);
  if (modelSelect) modelSelect.value = customModelId;
  refreshAlgorithmSelect(customModelId, algo);
  syncAlgorithmSelects(algo);

  const nextPlane2d = !!cfg.plane2d;
  if (panelPlane2dToggle) panelPlane2dToggle.checked = nextPlane2d;
  if (plane2dToggle) plane2dToggle.checked = nextPlane2d;

  renderClusters();
  renderGroupColors();

  if (runNow) {
    resetSimulation(customModelId, algo);
    if (sim && plane2dToggle) {
      sim.set_plane_2d(plane2dToggle.checked);
    }
    applyGroupColorsToMeshes();
  }
}

function addCluster(preset) {
  const idx = clusters.length + 1;
  clusters.push(
    preset ?? {
      name: `Sphere ${idx}`,
      count: 16,
      radius: 4,
      center: { x: idx * 2, y: 0, z: 0 },
      velocity: { x: 0, y: 0, z: 0 },
      radialSpeed: 0,
      drag: 0.08,
      group: idx - 1,
    }
  );
  renderClusters();
}

function buildCustomConfig(algorithmId) {
  const dt = Number(dtInput?.value) || customDt;
  customDt = dt;
  const cfg = {
    dt,
    algorithm: algorithmId,
    plane2d: plane2dToggle?.checked || false,
    clusters: clusters.map((c) => ({
      shape: "sphere",
      count: Number(c.count || 0),
      center: [Number(c.center.x || 0), Number(c.center.y || 0), Number(c.center.z || 0)],
      radius: Number(c.radius || 0),
      velocity: [Number(c.velocity.x || 0), Number(c.velocity.y || 0), Number(c.velocity.z || 0)],
      radialSpeed: Number(c.radialSpeed || 0),
      drag: Number(c.drag || 0),
      group: Number(c.group || 0),
    })),
  };
  return cfg;
}

function resetSimulation(modelId, algorithmId) {
  const targetModel = modelId || currentModelId();
  const targetAlgo = algorithmId || currentAlgorithmId(targetModel);
  let actualModel = targetModel;
  let actualAlgo = targetAlgo;
  try {
    if (targetModel === customModelId) {
      const cfg = buildCustomConfig(targetAlgo);
      sim = WasmSim.newFromConfig(cfg);
    } else {
      sim = WasmSim.newWithIds(targetModel, targetAlgo);
    }
  } catch (err) {
    console.error("Failed to create simulation", err);
    if (targetModel === customModelId) {
      try {
        sim = WasmSim.newFromConfig({ dt: Number(dtInput?.value) || customDt, algorithm: targetAlgo, plane2d: plane2dToggle?.checked || false, clusters: [] });
        actualModel = customModelId;
        actualAlgo = targetAlgo;
      } catch (fallbackErr) {
        console.error("Failed to start empty custom simulation", fallbackErr);
        sim = null;
      }
    } else {
      try {
        sim = WasmSim.new_demo();
        actualModel = "ring-swarm";
        actualAlgo = "flocking";
      } catch (fallbackErr) {
        console.error("Failed to start fallback simulation", fallbackErr);
        sim = null;
      }
    }
  }
  if (!sim) return;
  activeModelId = actualModel;
  activeAlgorithmId = actualAlgo;
  if (plane2dToggle) {
    sim.set_plane_2d(plane2dToggle.checked);
  }
  rebuildMeshes();
  renderGroupColors();
}

if (pauseButton) {
  setPaused(false);
  pauseButton.addEventListener("click", () => {
    setPaused(!paused);
  });
}

if (restartButton) {
  restartButton.addEventListener("click", () => {
    resetSimulation();
  });
}

if (plane2dToggle) {
  plane2dToggle.addEventListener("change", () => {
    sim.set_plane_2d(plane2dToggle.checked);
    if (panelPlane2dToggle) {
      panelPlane2dToggle.checked = plane2dToggle.checked;
    }
  });
}

function openSetup(open) {
  if (!setupPanel) return;
  if (open) {
    setupPanel.classList.add("is-open");
  } else {
    setupPanel.classList.remove("is-open");
  }
}

if (setupToggleBtn) {
  setupToggleBtn.addEventListener("click", () => {
    const isOpen = setupPanel?.classList.contains("is-open");
    openSetup(!isOpen);
  });
}

if (closeSetupBtn) {
  closeSetupBtn.addEventListener("click", () => openSetup(false));
}

if (addClusterBtn) {
  addClusterBtn.addEventListener("click", () => addCluster());
}

if (applySetupBtn) {
  applySetupBtn.addEventListener("click", () => {
    const hasClusters = clusterCountTotal() > 0;
    const targetModel = hasClusters ? customModelId : (modelSelect?.value || currentModelId());
    const targetAlgo = algorithmSelectPanel?.value || algorithmSelect?.value || currentAlgorithmId(targetModel);
    const plane2dDesired = panelPlane2dToggle?.checked;

    // If user defined clusters, force model to custom for this run.
    if (hasClusters && modelSelect) {
      modelSelect.value = customModelId;
    }

    refreshAlgorithmSelect(targetModel, targetAlgo);
    syncAlgorithmSelects(targetAlgo);
    if (panelPlane2dToggle && plane2dToggle && plane2dDesired !== undefined) {
      plane2dToggle.checked = plane2dDesired;
    }

    // Always rebuild on Apply to honor initial conditions.
    if (hasClusters && clusterCountTotal() === 0) {
      addCluster();
      return;
    }
    resetSimulation(targetModel, targetAlgo);
    if (plane2dDesired !== undefined && sim) {
      sim.set_plane_2d(plane2dDesired);
    }

    openSetup(false);
  });
}

if (dtInput) {
  dtInput.value = customDt.toFixed(3);
  dtInput.addEventListener("input", () => {
    customDt = Number(dtInput.value) || customDt;
  });
}

if (panelPlane2dToggle) {
  panelPlane2dToggle.checked = plane2dToggle?.checked || false;
}

if (saveConfigBtn) {
  saveConfigBtn.addEventListener("click", () => {
    const cfg = buildSetupConfig();
    const blob = new Blob([JSON.stringify(cfg, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = "rphys-setup.json";
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
  });
}

if (loadConfigBtn && configFileInput) {
  loadConfigBtn.addEventListener("click", () => {
    configFileInput.value = "";
    configFileInput.click();
  });

  configFileInput.addEventListener("change", () => {
    const file = configFileInput.files && configFileInput.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const cfg = JSON.parse(String(reader.result || "{}"));
        applySetupConfig(cfg, true);
      } catch (err) {
        console.error("Failed to load config", err);
      }
    };
    reader.readAsText(file);
  });
}

// Initialize selects to custom model so setup matches initial spawn.
if (modelSelect) {
  modelSelect.value = customModelId;
}
refreshAlgorithmSelect(customModelId, algorithmSelectPanel?.value || algorithmSelect?.value || currentAlgorithmId(customModelId));
syncAlgorithmSelects(algorithmSelectPanel?.value || algorithmSelect?.value || currentAlgorithmId(customModelId));
renderClusters();
resetSimulation(customModelId, currentAlgorithmId(customModelId));

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const dragOffset = new THREE.Vector3();
const dragTarget = new THREE.Vector3();
const tmpVec = new THREE.Vector3();
const tmpProj = new THREE.Vector3();
let dragIndex = null;
let dragging = false;
let panning = false;
let orbiting = false;
const pickNdcRadius = 0.08;
let dragMesh = null;
let dragInput = null;
let panInput = null;
let orbitInput = null;

renderer.domElement.style.touchAction = "none";

function isInViewport(event) {
  if (!viewport) return false;
  if (typeof event.composedPath === "function") {
    return event.composedPath().includes(viewport);
  }
  return viewport.contains(event.target);
}

function onWheel(event) {
  if (!isInViewport(event)) return;
  event.preventDefault();
  const dx = event.deltaX || 0;
  const dy = event.deltaY || 0;
  const delta = Math.abs(dy) > Math.abs(dx) ? dy : dx;
  orbit.radius = clamp(
    orbit.radius + delta * orbitZoomSpeed,
    orbitMinRadius,
    orbitMaxRadius
  );
  updateCameraFromOrbit();
}

renderer.domElement.addEventListener("wheel", onWheel, { passive: false });
viewport.addEventListener("wheel", onWheel, { passive: false });
window.addEventListener("wheel", onWheel, { passive: false });

function updatePointer(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  pointer.set(x, y);
}

function pickBoid() {
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects(meshes, false);
  if (hits.length > 0) {
    return {
      index: hits[0].object.userData.index,
      point: hits[0].point,
    };
  }

  let best = null;
  let bestDist = pickNdcRadius * pickNdcRadius;
  for (let i = 0; i < meshes.length; i += 1) {
    tmpProj.copy(meshes[i].position).project(camera);
    const dx = tmpProj.x - pointer.x;
    const dy = tmpProj.y - pointer.y;
    const d2 = dx * dx + dy * dy;
    if (d2 < bestDist) {
      bestDist = d2;
      best = i;
    }
  }

  if (best === null) return null;
  return { index: best, point: meshes[best].position.clone() };
}

function startDrag(event, isPointer) {
  if (event.button !== 0) return;
  if (panning || orbiting) return;
  if (isPointer && event.isPrimary === false) return;
  event.preventDefault();
  updatePointer(event);
  const hit = pickBoid();
  if (!hit) return;

  dragIndex = hit.index;
  dragging = true;
  dragOffset.copy(hit.point).sub(meshes[dragIndex].position);
  dragTarget.copy(meshes[dragIndex].position);
  dragMesh = meshes[dragIndex];
  dragMesh.scale.setScalar(1.7);
  dragPlane.setFromNormalAndCoplanarPoint(
    camera.getWorldDirection(tmpVec),
    hit.point
  );
  if (isPointer && renderer.domElement.setPointerCapture) {
    renderer.domElement.setPointerCapture(event.pointerId);
  }
}

function moveDrag(event) {
  if (!dragging || dragIndex === null) return;
  updatePointer(event);
  raycaster.setFromCamera(pointer, camera);
  if (raycaster.ray.intersectPlane(dragPlane, dragTarget)) {
    dragTarget.sub(dragOffset);
  }
}

function endDrag(event, isPointer) {
  if (!dragging) return;
  event.preventDefault();
  dragging = false;
  dragIndex = null;
  if (dragMesh) {
    dragMesh.scale.setScalar(1.0);
    dragMesh = null;
  }
  if (isPointer && renderer.domElement.releasePointerCapture) {
    renderer.domElement.releasePointerCapture(event.pointerId);
  }
}

function startPan(event, isPointer) {
  if (event.button !== 1) return;
  if (dragging) return;
  if (orbiting) return;
  event.preventDefault();
  panning = true;
  panLast.set(event.clientX, event.clientY);
  if (isPointer && renderer.domElement.setPointerCapture) {
    renderer.domElement.setPointerCapture(event.pointerId);
  }
}

function movePan(event) {
  if (!panning) return;
  const dx = event.clientX - panLast.x;
  const dy = event.clientY - panLast.y;
  panLast.set(event.clientX, event.clientY);
  camera.getWorldDirection(panDir);
  panRight.crossVectors(panDir, camera.up).normalize();
  panUp.copy(camera.up).normalize();
  const scale = orbit.radius * panScale;
  const moveRight = panRight.multiplyScalar(-dx * scale);
  const moveUp = panUp.multiplyScalar(dy * scale);
  orbitTarget.add(moveRight).add(moveUp);
  updateCameraFromOrbit();
}

function endPan(event, isPointer) {
  if (!panning) return;
  event.preventDefault();
  panning = false;
  if (isPointer && renderer.domElement.releasePointerCapture) {
    renderer.domElement.releasePointerCapture(event.pointerId);
  }
}

function startOrbit(event, isPointer) {
  if (event.button !== 1) return;
  if (dragging) return;
  if (panning) return;
  event.preventDefault();
  orbiting = true;
  panLast.set(event.clientX, event.clientY);
  if (isPointer && renderer.domElement.setPointerCapture) {
    renderer.domElement.setPointerCapture(event.pointerId);
  }
}

function moveOrbit(event) {
  if (!orbiting) return;
  const dx = event.clientX - panLast.x;
  const dy = event.clientY - panLast.y;
  panLast.set(event.clientX, event.clientY);
  orbit.theta -= dx * orbitRotateSpeed;
  orbit.phi -= dy * orbitRotateSpeed;
  orbit.phi = clamp(orbit.phi, orbitMinPhi, orbitMaxPhi);
  updateCameraFromOrbit();
}

function endOrbit(event, isPointer) {
  if (!orbiting) return;
  event.preventDefault();
  orbiting = false;
  if (isPointer && renderer.domElement.releasePointerCapture) {
    renderer.domElement.releasePointerCapture(event.pointerId);
  }
}

function onPointerDown(event) {
  if (dragInput || panInput || orbitInput) return;
  if (getGizmoPointer(event)) return;
  if (event.button === 0) {
    dragInput = "pointer";
    startDrag(event, true);
    if (!dragging) dragInput = null;
  } else if (event.button === 1) {
    if (event.shiftKey) {
      orbitInput = "pointer";
      startOrbit(event, true);
      if (!orbiting) orbitInput = null;
    } else {
      panInput = "pointer";
      startPan(event, true);
      if (!panning) panInput = null;
    }
  }
}

function onPointerMove(event) {
  if (dragInput === "pointer") moveDrag(event);
  if (panInput === "pointer") movePan(event);
  if (orbitInput === "pointer") moveOrbit(event);
}

function onPointerUp(event) {
  if (dragInput === "pointer") {
    endDrag(event, true);
    dragInput = null;
  }
  if (panInput === "pointer") {
    endPan(event, true);
    panInput = null;
  }
  if (orbitInput === "pointer") {
    endOrbit(event, true);
    orbitInput = null;
  }
}

function onMouseDown(event) {
  if (dragInput || panInput || orbitInput) return;
  if (getGizmoPointer(event)) return;
  if (event.button === 0) {
    dragInput = "mouse";
    startDrag(event, false);
    if (!dragging) dragInput = null;
  } else if (event.button === 1) {
    if (event.shiftKey) {
      orbitInput = "mouse";
      startOrbit(event, false);
      if (!orbiting) orbitInput = null;
    } else {
      panInput = "mouse";
      startPan(event, false);
      if (!panning) panInput = null;
    }
  }
}

function onMouseMove(event) {
  if (dragInput === "mouse") moveDrag(event);
  if (panInput === "mouse") movePan(event);
  if (orbitInput === "mouse") moveOrbit(event);
}

function onMouseUp(event) {
  if (dragInput === "mouse") {
    endDrag(event, false);
    dragInput = null;
  }
  if (panInput === "mouse") {
    endPan(event, false);
    panInput = null;
  }
  if (orbitInput === "mouse") {
    endOrbit(event, false);
    orbitInput = null;
  }
}

renderer.domElement.addEventListener("pointerdown", onPointerDown);
renderer.domElement.addEventListener("pointermove", onPointerMove);
renderer.domElement.addEventListener("pointerup", onPointerUp);
renderer.domElement.addEventListener("pointercancel", onPointerUp);
renderer.domElement.addEventListener("mousedown", onMouseDown);
renderer.domElement.addEventListener("dblclick", onGizmoDoubleClick);
renderer.domElement.addEventListener("mousemove", onMouseMove);
renderer.domElement.addEventListener("mouseup", onMouseUp);
window.addEventListener("pointermove", onPointerMove);
window.addEventListener("pointerup", onPointerUp);
window.addEventListener("pointercancel", onPointerUp);
window.addEventListener("mousemove", onMouseMove);
window.addEventListener("mouseup", onMouseUp);
renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());
renderer.domElement.addEventListener("auxclick", (event) => event.preventDefault());

function renderGizmo() {
  gizmoRoot.quaternion.copy(camera.quaternion).invert();
  gizmoRenderer.render(gizmoScene, gizmoCamera);
}

function animate() {
  if (!sim) {
    requestAnimationFrame(animate);
    return;
  }
  if (dragging && dragIndex !== null) {
    sim.set_position_and_velocity(
      dragIndex,
      dragTarget.x,
      dragTarget.y,
      dragTarget.z,
      0.0,
      0.0,
      0.0
    );
  }

  if (!paused) {
    sim.tick();
  }

  if (dragging && dragIndex !== null) {
    sim.set_position_and_velocity(
      dragIndex,
      dragTarget.x,
      dragTarget.y,
      dragTarget.z,
      0.0,
      0.0,
      0.0
    );
  }

  const positions = sim.positions();
  const count = meshes.length;
  for (let i = 0; i < count; i += 1) {
    const base = i * 3;
    meshes[i].position.set(
      positions[base + 0],
      positions[base + 1],
      positions[base + 2]
    );
  }

  renderer.getSize(gizmoViewportSize);
  const width = gizmoViewportSize.x;
  const height = gizmoViewportSize.y;
  renderer.setViewport(0, 0, width, height);
  renderer.setScissorTest(false);
  renderer.clear();
  renderer.render(scene, camera);
  renderGizmo();
  requestAnimationFrame(animate);
}

animate();

function resizeRenderer() {
  const width = viewport.clientWidth;
  const height = viewport.clientHeight;
  if (width === 0 || height === 0) return;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  gizmoRenderer.setSize(gizmoBufferSize, gizmoBufferSize, false);
}

window.addEventListener("resize", resizeRenderer);
resizeRenderer();
