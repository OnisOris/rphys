import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import init, * as rphysWasm from "./pkg/rphys.js";

await init();

const { WasmSim, available_models, available_algorithms, algorithms_for_model } =
  rphysWasm;
const flocking_defaults = rphysWasm.flocking_defaults;
const flocking_alpha_defaults = rphysWasm.flocking_alpha_defaults;
const formation_ecbf_defaults = rphysWasm.formation_ecbf_defaults;
const safe_flocking_alpha_defaults = rphysWasm.safe_flocking_alpha_defaults;

const RECORD_FIELDS_BASE = ["x", "y", "z", "vx", "vy", "vz"];
const RECORD_FIELDS_SAFE_FLOCKING = [
  ...RECORD_FIELDS_BASE,
  "unx",
  "uny",
  "unz",
  "ux",
  "uy",
  "uz",
  "slack",
  "active",
  "constraints",
];
const defaultStateFields = RECORD_FIELDS_BASE;
const fallbackFlockParams = {
  neighbor_radius: 2.6,
  separation_radius: 0.9,
  cohesion_weight: 0.45,
  alignment_weight: 0.65,
  separation_weight: 10.35,
  boundary_radius: 6.0,
  boundary_weight: 0.8,
  max_speed: 2.4,
  max_force: 1.6,
  speed_limit: 2.0,
};

const rawFlockDefaults =
  typeof flocking_defaults === "function" ? flocking_defaults() : null;
const flockParamDefaults = buildFlockParamsDefaults(
  rawFlockDefaults,
  fallbackFlockParams
);

const fallbackFlockAlphaParams = {
  neighbor_radius: 2.6,
  desired_distance: 1.4,
  sigma_eps: 0.1,
  bump_h: 0.2,
  phi_a: 5.0,
  phi_b: 5.0,
  alpha_weight: 1.0,
  alignment_weight: 0.65,
  boundary_radius: 6.0,
  boundary_weight: 0.8,
  max_speed: 2.4,
  max_force: 1.6,
  speed_limit: 2.0,
};

const rawFlockAlphaDefaults =
  typeof flocking_alpha_defaults === "function" ? flocking_alpha_defaults() : null;
const flockAlphaParamDefaults = buildFlockAlphaParamsDefaults(
  rawFlockAlphaDefaults,
  fallbackFlockAlphaParams
);

const fallbackFormationEcbfParams = {
  k1: 2.0,
  k2: 3.0,
  gamma1: 0.5,
  gamma2: 0.5,
  m1: 1.0,
  m2: 2.0,
  obs_k1: 1.0,
  obs_k2: 1.0,
  obs_k3: 2.0,
  obs_a1: 2.0,
  obs_a2: 1.0,
  obs_b1: 1.0,
  obs_b2: 2.0,
  do_kappa1: 4.0,
  do_kappa2: 2.0,
  do_kappa3: 3.0,
  do_eta1: 1.5,
  do_eta2: 1.5,
  do_eta3: 0.5,
  do_n1: 0.5,
  do_n2: 1.5,
  delta_theta: 0.2,
  delta2_star: 0.0,
  lambda1: 2.0,
  lambda2: 2.0,
  gravity: 9.81,
  desired_yaw: 0.0,
  smooth_eps: 0.01,
  mu_dot_filter: 0.8,
  alpha_dot_filter: 0.8,
  u_min: [-6.0, -6.0, 0.0],
  u_max: [6.0, 6.0, 20.0],
  qp_iters: 12,
  obstacles: [
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [47, 86, 10], d: 5 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [52, 78, 9], d: 4 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [43, 82, 61.5], d: 5 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [49, 75, 60.5], d: 5.5 },
    { a2: [0, 0.001, 0], a1: [-0.06, 0, -0.089], a0: [95, 15, 100], d: 3 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [69, 83, 124.5], d: 6 },
  ],
  formation_offsets: [
    [-3, 3, 0],
    [-3, -3, 0],
    [3, 3, 0],
    [3, -3, 0],
  ],
  auto_offsets: true,
  adjacency: [
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
  ],
  leader_links: [1, 1, 0, 0],
  leader: { kind: "circle", center: [0, 0, 0], radius: 6, omega: 0.2 },
  leader_time_scale: 1.0,
  leader_paused: false,
  use_moving_obstacle_terms: true,
};

const rawFormationDefaults =
  typeof formation_ecbf_defaults === "function" ? formation_ecbf_defaults() : null;
const formationEcbfParamDefaults = normalizeFormationEcbfParams(
  rawFormationDefaults,
  fallbackFormationEcbfParams
);

const fallbackSafeFlockingAlphaParams = {
  neighbor_radius: 2.6,
  desired_distance: 1.4,
  sigma_eps: 0.1,
  bump_h: 0.2,
  phi_a: 5.0,
  phi_b: 5.0,
  alpha_weight: 1.0,
  alignment_weight: 0.65,
  boundary_radius: 6.0,
  boundary_weight: 0.8,
  max_speed: 2.4,
  max_force: 1.6,
  speed_limit: 2.0,
  use_obstacles: true,
  use_agent_cbf: true,
  agent_safe_distance: 0.9,
  cbf_neighbor_radius: 2.6,
  lambda1: 2.0,
  lambda2: 2.0,
  delta_theta: 0.2,
  delta2_star: 0.0,
  use_moving_obstacle_terms: true,
  two_pass: false,
  u_min: [-6.0, -6.0, -6.0],
  u_max: [6.0, 6.0, 6.0],
  slack_weight: 50.0,
  slack_max: 50.0,
  smooth_eps: 0.01,
  qp_iters: 14,
  obstacles: [
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [47, 86, 10], d: 5 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [52, 78, 9], d: 4 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [43, 82, 61.5], d: 5 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [49, 75, 60.5], d: 5.5 },
    { a2: [0, 0.001, 0], a1: [-0.06, 0, -0.089], a0: [95, 15, 100], d: 3 },
    { a2: [0, 0, 0], a1: [0, 0, 0], a0: [69, 83, 124.5], d: 6 },
  ],
};

const rawSafeFlockingAlphaDefaults =
  typeof safe_flocking_alpha_defaults === "function"
    ? safe_flocking_alpha_defaults()
    : null;
const safeFlockingAlphaParamDefaults = normalizeFormationEcbfParams(
  rawSafeFlockingAlphaDefaults,
  fallbackSafeFlockingAlphaParams
);

const algorithmParamDefinitions = {
  flocking: {
    label: "Flocking",
    params: [
      { key: "neighbor_radius", label: "Neighbor radius (m)", step: 0.1, min: 0 },
      { key: "separation_radius", label: "Separation radius (m)", step: 0.1, min: 0 },
      { key: "cohesion_weight", label: "Cohesion weight (unitless)", step: 0.05, min: 0 },
      { key: "alignment_weight", label: "Alignment weight (unitless)", step: 0.05, min: 0 },
      { key: "separation_weight", label: "Separation weight (unitless)", step: 0.1, min: 0 },
      { key: "boundary_radius", label: "Boundary radius (m)", step: 0.1, min: 0 },
      { key: "boundary_weight", label: "Boundary weight (unitless)", step: 0.05, min: 0 },
      { key: "max_speed", label: "Max speed (m/s)", step: 0.1, min: 0 },
      { key: "max_force", label: "Max force (N)", step: 0.1, min: 0 },
      { key: "speed_limit", label: "Speed limit gain (1/s)", step: 0.1, min: 0 },
    ],
  },
  "flocking-alpha": {
    label: "Flocking alpha-lattice",
    params: [
      { key: "neighbor_radius", label: "Neighbor radius r (m)", step: 0.1, min: 0 },
      { key: "desired_distance", label: "Desired distance d (m)", step: 0.1, min: 0 },
      { key: "sigma_eps", label: "Sigma epsilon (unitless)", step: 0.01, min: 0 },
      { key: "bump_h", label: "Bump h (0..1)", step: 0.01, min: 0 },
      { key: "phi_a", label: "Phi a (unitless)", step: 0.1, min: 0 },
      { key: "phi_b", label: "Phi b (unitless)", step: 0.1, min: 0 },
      { key: "alpha_weight", label: "Alpha weight (unitless)", step: 0.05, min: 0 },
      { key: "alignment_weight", label: "Alignment weight (unitless)", step: 0.05, min: 0 },
      { key: "boundary_radius", label: "Boundary radius (m)", step: 0.1, min: 0 },
      { key: "boundary_weight", label: "Boundary weight (unitless)", step: 0.05, min: 0 },
      { key: "max_speed", label: "Max speed (m/s)", step: 0.1, min: 0 },
      { key: "max_force", label: "Max force (N)", step: 0.1, min: 0 },
      { key: "speed_limit", label: "Speed limit gain (1/s)", step: 0.1, min: 0 },
    ],
  },
  "formation-ecbf": {
    label: "Fixed-time formation + ECBF",
    params: [
      { key: "k1", label: "k1 (position gain)", step: 0.1, min: 0 },
      { key: "k2", label: "k2 (velocity gain)", step: 0.1, min: 0 },
      { key: "gamma1", label: "gamma1", step: 0.05, min: 0 },
      { key: "gamma2", label: "gamma2", step: 0.05, min: 0 },
      { key: "m1", label: "m1", step: 0.1, min: 0 },
      { key: "m2", label: "m2", step: 0.1, min: 0.1 },
      { key: "lambda1", label: "lambda1", step: 0.1, min: 0 },
      { key: "lambda2", label: "lambda2", step: 0.1, min: 0 },
      { key: "delta_theta", label: "delta_theta", step: 0.05, min: 0 },
      { key: "delta2_star", label: "delta2_star", step: 0.01, min: 0 },
      { key: "smooth_eps", label: "smooth_eps", step: 0.001, min: 0 },
      { key: "mu_dot_filter", label: "mu_dot_filter", step: 0.05, min: 0, max: 0.99 },
      { key: "alpha_dot_filter", label: "alpha_dot_filter", step: 0.05, min: 0, max: 0.99 },
      { key: "desired_yaw", label: "desired_yaw (rad)", step: 0.1 },
      { key: "gravity", label: "gravity (m/s^2)", step: 0.1, min: 0 },
      { key: "leader_time_scale", label: "leader_time_scale", step: 0.1, min: 0 },
      { key: "u_min_x", label: "u_min x", step: 0.1, path: ["u_min", 0] },
      { key: "u_min_y", label: "u_min y", step: 0.1, path: ["u_min", 1] },
      { key: "u_min_z", label: "u_min z", step: 0.1, path: ["u_min", 2] },
      { key: "u_max_x", label: "u_max x", step: 0.1, path: ["u_max", 0] },
      { key: "u_max_y", label: "u_max y", step: 0.1, path: ["u_max", 1] },
      { key: "u_max_z", label: "u_max z", step: 0.1, path: ["u_max", 2] },
    ],
  },
  "safe-flocking-alpha": {
    label: "Safe flocking (alpha + CBF-QP)",
    params: [
      { key: "neighbor_radius", label: "Neighbor radius r (m)", step: 0.1, min: 0 },
      { key: "desired_distance", label: "Desired distance d (m)", step: 0.1, min: 0 },
      { key: "sigma_eps", label: "Sigma epsilon (unitless)", step: 0.01, min: 0 },
      { key: "bump_h", label: "Bump h (0..1)", step: 0.01, min: 0, max: 0.999 },
      { key: "phi_a", label: "Phi a (unitless)", step: 0.1, min: 0 },
      { key: "phi_b", label: "Phi b (unitless)", step: 0.1, min: 0 },
      { key: "alpha_weight", label: "Alpha weight", step: 0.05, min: 0 },
      { key: "alignment_weight", label: "Alignment weight", step: 0.05, min: 0 },
      { key: "boundary_radius", label: "Boundary radius (m)", step: 0.1, min: 0 },
      { key: "boundary_weight", label: "Boundary weight", step: 0.05, min: 0 },
      { key: "max_speed", label: "Max speed (m/s)", step: 0.1, min: 0 },
      { key: "max_force", label: "Max force", step: 0.1, min: 0 },
      { key: "speed_limit", label: "Speed limit gain (1/s)", step: 0.1, min: 0 },
      { key: "agent_safe_distance", label: "Agent safe distance d_safe (m)", step: 0.05, min: 0 },
      { key: "cbf_neighbor_radius", label: "CBF neighbor radius (m)", step: 0.1, min: 0 },
      { key: "lambda1", label: "CBF lambda1", step: 0.1, min: 0 },
      { key: "lambda2", label: "CBF lambda2", step: 0.1, min: 0 },
      { key: "delta_theta", label: "Robust delta_theta", step: 0.05, min: 0 },
      { key: "delta2_star", label: "Robust delta2_star", step: 0.01, min: 0, max: 0.99 },
      { key: "smooth_eps", label: "Smooth eps", step: 0.001, min: 0 },
      { key: "slack_weight", label: "Slack weight", step: 1, min: 0 },
      { key: "slack_max", label: "Slack max", step: 0.5, min: 0 },
      { key: "qp_iters", label: "QP iters", step: 1, min: 1 },
      { key: "u_min_x", label: "u_min x", step: 0.1, path: ["u_min", 0] },
      { key: "u_min_y", label: "u_min y", step: 0.1, path: ["u_min", 1] },
      { key: "u_min_z", label: "u_min z", step: 0.1, path: ["u_min", 2] },
      { key: "u_max_x", label: "u_max x", step: 0.1, path: ["u_max", 0] },
      { key: "u_max_y", label: "u_max y", step: 0.1, path: ["u_max", 1] },
      { key: "u_max_z", label: "u_max z", step: 0.1, path: ["u_max", 2] },
    ],
  },
};

const algorithmParamsState = {
  flocking: { ...flockParamDefaults },
  "flocking-alpha": { ...flockAlphaParamDefaults },
  "formation-ecbf":
    typeof structuredClone === "function"
      ? structuredClone(formationEcbfParamDefaults)
      : JSON.parse(JSON.stringify(formationEcbfParamDefaults)),
  "safe-flocking-alpha":
    typeof structuredClone === "function"
      ? structuredClone(safeFlockingAlphaParamDefaults)
      : JSON.parse(JSON.stringify(safeFlockingAlphaParamDefaults)),
};

const viewport = document.getElementById("viewport");
const homeButton = document.getElementById("home-btn");
const pauseButton = document.getElementById("sim-toggle-btn");
const restartButton = document.getElementById("sim-restart-btn");
const modelSelect = document.getElementById("model-select");
const algorithmSelect = document.getElementById("algorithm-select");
const setupToggleBtn = document.getElementById("setup-toggle-btn");
const setupPanel = document.getElementById("setup-panel");
const setupTabButtons = document.querySelectorAll("[data-setup-tab]");
const setupTabContents = document.querySelectorAll("[data-setup-content]");
const applySetupBtn = document.getElementById("apply-setup-btn");
const closeSetupBtn = document.getElementById("close-setup-btn");
const dtInput = document.getElementById("dt-input");
const clusterList = document.getElementById("cluster-list");
const addClusterBtn = document.getElementById("add-cluster-btn");
const agentTotalEl = document.getElementById("agent-total");
const algorithmSelectPanel = document.getElementById("algorithm-select-panel");
const panelPlane2dToggle = document.getElementById("panel-plane2d-toggle");
const groupColorList = document.getElementById("group-color-list");
const algorithmParamsPanel = document.getElementById("algorithm-params");
const algorithmParamsEmpty = document.getElementById("algorithm-params-empty");
const saveConfigBtn = document.getElementById("save-config-btn");
const loadConfigBtn = document.getElementById("load-config-btn");
const configFileInput = document.getElementById("config-file-input");
const recordToggle = document.getElementById("record-toggle");
const recordStrideInput = document.getElementById("record-stride-input");
const recordMaxFramesInput = document.getElementById("record-max-frames-input");
const recordFastTimeInput = document.getElementById("record-fast-time-input");
const recordFastBtn = document.getElementById("record-fast-btn");
const recordDownloadBtn = document.getElementById("record-download-btn");
const recordClearBtn = document.getElementById("record-clear-btn");
const recordStatus = document.getElementById("record-status");
const playbackLoadBtn = document.getElementById("playback-load-btn");
const playbackExitBtn = document.getElementById("playback-exit-btn");
const playbackSeekBtn = document.getElementById("playback-seek-btn");
const playbackGoBtn = document.getElementById("playback-go-btn");
const playbackFileInput = document.getElementById("playback-file-input");
const playbackTimeInput = document.getElementById("playback-time-input");
const playbackSlider = document.getElementById("playback-slider");
const playbackStatus = document.getElementById("playback-status");

const ECBF_ALGO_ID = "formation-ecbf";
const QUADROTOR_MODEL_ID = "quadrotor-swarm";

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

const attitudeGroup = new THREE.Group();
attitudeGroup.visible = false;
scene.add(attitudeGroup);

const leaderMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.08, 14, 14),
  new THREE.MeshStandardMaterial({ color: 0xff3b30, emissive: 0x330000 })
);
leaderMarker.visible = false;
scene.add(leaderMarker);

const leaderPathMaterial = new THREE.LineBasicMaterial({
  color: 0xff6b6b,
  transparent: true,
  opacity: 0.65,
});
const leaderPathLine = new THREE.Line(new THREE.BufferGeometry(), leaderPathMaterial);
leaderPathLine.visible = false;
scene.add(leaderPathLine);

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
let recording = false;
let recordFrames = [];
let recordStep = 0;
let recordMeta = null;
let fastExportRunning = false;
let playbackActive = false;
let playbackData = null;
let playbackIndex = 0;

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

function preferredModelForAlgorithm(algorithmId) {
  if (algorithmId === ECBF_ALGO_ID) {
    const hasQuad = modelCatalog.some((m) => m.id === QUADROTOR_MODEL_ID);
    return hasQuad ? QUADROTOR_MODEL_ID : customModelId;
  }
  return null;
}

function handleAlgorithmChange(targetAlgo) {
  const preferredModel = preferredModelForAlgorithm(targetAlgo);
  const current = currentModelId();
  if (preferredModel && current !== preferredModel) {
    if (modelSelect) modelSelect.value = preferredModel;
    refreshAlgorithmSelect(preferredModel, targetAlgo);
    resetSimulation(preferredModel, targetAlgo);
    renderAlgorithmParamsPanel();
    return;
  }

  const targetModel = currentModelId();
  if (activeModelId && targetModel === activeModelId && sim) {
    try {
      sim.set_algorithm(targetAlgo);
      activeAlgorithmId = targetAlgo;
      applyAlgorithmParamsToSim(targetAlgo);
      renderAlgorithmParamsPanel();
      return;
    } catch (err) {
      console.error("set_algorithm failed, recreating sim", err);
    }
  }
  resetSimulation();
  renderAlgorithmParamsPanel();
}

if (modelSelect) {
  populateSelect(modelSelect, modelCatalog, (m) => m.id, (m) => m.name);
  modelSelect.addEventListener("change", () => {
    refreshAlgorithmSelect(modelSelect.value);
    resetSimulation();
    renderAlgorithmParamsPanel();
  });
}

if (algorithmSelect) {
  refreshAlgorithmSelect(currentModelId());
  algorithmSelect.addEventListener("change", () => {
    if (algorithmSelectPanel) algorithmSelectPanel.value = algorithmSelect.value;
    const targetAlgo = algorithmSelect.value;
    handleAlgorithmChange(targetAlgo);
  });
}

if (algorithmSelectPanel) {
  populateSelect(algorithmSelectPanel, algorithmCatalog, (a) => a.id, (a) => a.name);
  algorithmSelectPanel.addEventListener("change", () => {
    if (algorithmSelect) {
      algorithmSelect.value = algorithmSelectPanel.value;
    }
    const targetAlgo = algorithmSelectPanel.value;
    handleAlgorithmChange(targetAlgo);
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
let simTime = 0;

function updatePlaybackControls() {
  if (!playbackGoBtn) return;
  if (!playbackActive || !playbackData) {
    playbackGoBtn.disabled = true;
    playbackGoBtn.textContent = "Go";
    return;
  }
  playbackGoBtn.disabled = false;
  playbackGoBtn.textContent = paused ? "Go" : "Pause";
}

function setPaused(next) {
  paused = next;
  if (pauseButton) {
    pauseButton.dataset.state = paused ? "paused" : "running";
    pauseButton.textContent = paused ? "Resume" : "Pause";
    pauseButton.setAttribute("aria-pressed", paused ? "true" : "false");
  }
  updatePlaybackControls();
}

const geometry = new THREE.SphereGeometry(0.12, 16, 16);
const baseMaterial = new THREE.MeshStandardMaterial({
  color: 0x6ad1ff,
  roughness: 0.35,
  metalness: 0.1,
});

const meshes = [];
const attitudeHelpers = [];

let leaderPathSignature = "";
let leaderPathTime = 0;
let leaderHoldState = null;

function refreshGroups() {
  currentGroups = [];
  if (playbackActive && playbackData) {
    if (Array.isArray(playbackData.groups) && playbackData.groups.length > 0) {
      currentGroups = playbackData.groups.slice();
    } else {
      currentGroups = new Array(playbackData.agentCount).fill(0);
    }
    return;
  }
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
  attitudeHelpers.forEach((helper) => attitudeGroup.remove(helper));
  attitudeHelpers.length = 0;
  const count = playbackActive && playbackData ? playbackData.agentCount : (sim ? sim.len() : 0);
  refreshGroups();
  for (let i = 0; i < count; i += 1) {
    const groupId = currentGroups[i] ?? 0;
    const material = baseMaterial.clone();
    material.color.set(colorForGroup(groupId));
    const mesh = new THREE.Mesh(geometry, material);
    mesh.userData.index = i;
    scene.add(mesh);
    meshes.push(mesh);

    const axes = new THREE.AxesHelper(0.35);
    axes.visible = false;
    attitudeGroup.add(axes);
    attitudeHelpers.push(axes);
  }
}

function clusterCountTotal() {
  return clusters.reduce((sum, c) => sum + Number(c.count || 0), 0);
}

function updateAgentTotal() {
  if (!agentTotalEl) return;
  if (playbackActive && playbackData) {
    agentTotalEl.textContent = `${playbackData.agentCount} agents (playback)`;
  } else {
    agentTotalEl.textContent = `${clusterCountTotal()} agents`;
  }
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

function makeNumberField(label, value, onChange, options) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";
  const lab = document.createElement("label");
  lab.textContent = label;
  const input = document.createElement("input");
  input.type = "number";
  if (options?.step !== undefined) input.step = String(options.step);
  if (options?.min !== undefined) input.min = String(options.min);
  if (options?.max !== undefined) input.max = String(options.max);
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

function makeSelectField(label, options, value, onChange) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";
  const lab = document.createElement("label");
  lab.textContent = label;
  const select = document.createElement("select");
  options.forEach((opt) => {
    const option = document.createElement("option");
    option.value = opt.value;
    option.textContent = opt.label;
    if (opt.value === value) option.selected = true;
    select.append(option);
  });
  select.addEventListener("change", () => onChange(select.value));
  wrapper.append(lab, select);
  return wrapper;
}

function makeToggleField(label, checked, onChange) {
  const wrapper = document.createElement("div");
  wrapper.className = "field";
  const lab = document.createElement("label");
  lab.textContent = label;
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = !!checked;
  input.addEventListener("change", () => onChange(input.checked));
  wrapper.append(lab, input);
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

function readRecordStride() {
  const stride = Math.round(Number(recordStrideInput?.value) || 1);
  return Math.max(1, stride);
}

function readRecordMaxFrames() {
  const maxFrames = Math.round(Number(recordMaxFramesInput?.value) || 0);
  return Math.max(0, maxFrames);
}

function readFastTime() {
  const time = Number(recordFastTimeInput?.value) || 0;
  return Math.max(0, time);
}

function currentDt() {
  if (sim && typeof sim.dt === "function") {
    const dt = sim.dt();
    if (Number.isFinite(dt)) return dt;
  }
  return Number(dtInput?.value) || customDt;
}

function coerceNumber(value, fallback) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function buildFlockParamsDefaults(raw, fallback) {
  return {
    neighbor_radius: coerceNumber(raw?.neighbor_radius, fallback.neighbor_radius),
    separation_radius: coerceNumber(raw?.separation_radius, fallback.separation_radius),
    cohesion_weight: coerceNumber(raw?.cohesion_weight, fallback.cohesion_weight),
    alignment_weight: coerceNumber(raw?.alignment_weight, fallback.alignment_weight),
    separation_weight: coerceNumber(raw?.separation_weight, fallback.separation_weight),
    boundary_radius: coerceNumber(raw?.boundary_radius, fallback.boundary_radius),
    boundary_weight: coerceNumber(raw?.boundary_weight, fallback.boundary_weight),
    max_speed: coerceNumber(raw?.max_speed, fallback.max_speed),
    max_force: coerceNumber(raw?.max_force, fallback.max_force),
    speed_limit: coerceNumber(raw?.speed_limit, fallback.speed_limit),
  };
}

function normalizeFlockParams(raw) {
  return buildFlockParamsDefaults(raw, flockParamDefaults);
}

function buildFlockAlphaParamsDefaults(raw, fallback) {
  return {
    neighbor_radius: coerceNumber(raw?.neighbor_radius, fallback.neighbor_radius),
    desired_distance: coerceNumber(raw?.desired_distance, fallback.desired_distance),
    sigma_eps: coerceNumber(raw?.sigma_eps, fallback.sigma_eps),
    bump_h: coerceNumber(raw?.bump_h, fallback.bump_h),
    phi_a: coerceNumber(raw?.phi_a, fallback.phi_a),
    phi_b: coerceNumber(raw?.phi_b, fallback.phi_b),
    alpha_weight: coerceNumber(raw?.alpha_weight, fallback.alpha_weight),
    alignment_weight: coerceNumber(raw?.alignment_weight, fallback.alignment_weight),
    boundary_radius: coerceNumber(raw?.boundary_radius, fallback.boundary_radius),
    boundary_weight: coerceNumber(raw?.boundary_weight, fallback.boundary_weight),
    max_speed: coerceNumber(raw?.max_speed, fallback.max_speed),
    max_force: coerceNumber(raw?.max_force, fallback.max_force),
    speed_limit: coerceNumber(raw?.speed_limit, fallback.speed_limit),
  };
}

function normalizeFlockAlphaParams(raw) {
  return buildFlockAlphaParamsDefaults(raw, flockAlphaParamDefaults);
}

function normalizeFormationEcbfParams(raw, fallback) {
  const base = JSON.parse(JSON.stringify(fallback || {}));
  if (!raw || typeof raw !== "object") return base;
  Object.entries(raw).forEach(([key, value]) => {
    if (Array.isArray(value)) {
      base[key] = value.map((item) =>
        Array.isArray(item) ? item.slice() : item
      );
      return;
    }
    if (value && typeof value === "object") {
      base[key] = { ...(base[key] || {}), ...value };
      return;
    }
    if (typeof value === "number" && Number.isFinite(value)) {
      base[key] = value;
      return;
    }
    if (typeof value === "boolean") {
      base[key] = value;
    }
  });
  return base;
}

function getParamValue(params, param) {
  if (!param?.path) return params[param.key];
  let cur = params;
  for (const step of param.path) {
    if (cur == null) return undefined;
    cur = cur[step];
  }
  return cur;
}

function setParamValue(params, param, value) {
  if (!param?.path) {
    params[param.key] = value;
    return;
  }
  let cur = params;
  const last = param.path[param.path.length - 1];
  for (let i = 0; i < param.path.length - 1; i += 1) {
    const key = param.path[i];
    if (cur[key] == null) {
      cur[key] = typeof param.path[i + 1] === "number" ? [] : {};
    }
    cur = cur[key];
  }
  cur[last] = value;
}

function ensureAlgorithmParams(algorithmId) {
  if (!algorithmId) return {};
  if (!algorithmParamsState[algorithmId]) {
    if (algorithmId === "flocking") {
      algorithmParamsState[algorithmId] = { ...flockParamDefaults };
    } else if (algorithmId === "flocking-alpha") {
      algorithmParamsState[algorithmId] = { ...flockAlphaParamDefaults };
    } else if (algorithmId === "formation-ecbf") {
      algorithmParamsState[algorithmId] =
        typeof structuredClone === "function"
          ? structuredClone(formationEcbfParamDefaults)
          : JSON.parse(JSON.stringify(formationEcbfParamDefaults));
    } else if (algorithmId === "safe-flocking-alpha") {
      algorithmParamsState[algorithmId] =
        typeof structuredClone === "function"
          ? structuredClone(safeFlockingAlphaParamDefaults)
          : JSON.parse(JSON.stringify(safeFlockingAlphaParamDefaults));
    } else {
      algorithmParamsState[algorithmId] = {};
    }
  }
  return algorithmParamsState[algorithmId];
}

function buildAlgorithmParamsConfig() {
  const out = {};
  Object.entries(algorithmParamsState).forEach(([algoId, params]) => {
    if (params && typeof params === "object") {
      out[algoId] = { ...params };
    }
  });
  return out;
}

function loadAlgorithmParamsConfig(raw) {
  if (!raw || typeof raw !== "object") return;
  Object.entries(raw).forEach(([algoId, params]) => {
    if (!params || typeof params !== "object") return;
    if (algoId === "flocking") {
      algorithmParamsState[algoId] = normalizeFlockParams(params);
    } else if (algoId === "flocking-alpha") {
      algorithmParamsState[algoId] = normalizeFlockAlphaParams(params);
    } else if (algoId === "formation-ecbf") {
      algorithmParamsState[algoId] = normalizeFormationEcbfParams(
        params,
        formationEcbfParamDefaults
      );
    } else if (algoId === "safe-flocking-alpha") {
      algorithmParamsState[algoId] = normalizeFormationEcbfParams(
        params,
        safeFlockingAlphaParamDefaults
      );
    } else {
      algorithmParamsState[algoId] = { ...params };
    }
  });
}

function applyAlgorithmParamsToSim(algorithmId, simOverride) {
  const targetSim = simOverride || sim;
  if (!targetSim || !algorithmId) return;
  if (algorithmId === "flocking" && typeof targetSim.set_flock_params === "function") {
    const params = ensureAlgorithmParams(algorithmId);
    try {
      targetSim.set_flock_params(params);
    } catch (err) {
      console.error("set_flock_params failed", err);
    }
  } else if (
    algorithmId === "flocking-alpha" &&
    typeof targetSim.set_flock_alpha_params === "function"
  ) {
    const params = ensureAlgorithmParams(algorithmId);
    try {
      targetSim.set_flock_alpha_params(params);
    } catch (err) {
      console.error("set_flock_alpha_params failed", err);
    }
  } else if (
    algorithmId === "formation-ecbf" &&
    typeof targetSim.set_formation_ecbf_params === "function"
  ) {
    const params = ensureAlgorithmParams(algorithmId);
    try {
      targetSim.set_formation_ecbf_params(params);
    } catch (err) {
      console.error("set_formation_ecbf_params failed", err);
    }
  } else if (
    algorithmId === "safe-flocking-alpha" &&
    typeof targetSim.set_safe_flocking_alpha_params === "function"
  ) {
    const params = ensureAlgorithmParams(algorithmId);
    try {
      targetSim.set_safe_flocking_alpha_params(params);
    } catch (err) {
      console.error("set_safe_flocking_alpha_params failed", err);
    }
  }
}

function renderAlgorithmParamsPanel() {
  if (!algorithmParamsPanel) return;
  algorithmParamsPanel.innerHTML = "";
  const algorithmId =
    algorithmSelectPanel?.value ||
    algorithmSelect?.value ||
    currentAlgorithmId(customModelId);
  const definition = algorithmParamDefinitions[algorithmId];
  if (!definition) {
    if (algorithmParamsEmpty) algorithmParamsEmpty.style.display = "block";
    return;
  }
  if (algorithmParamsEmpty) algorithmParamsEmpty.style.display = "none";
  const params = ensureAlgorithmParams(algorithmId);
  if (algorithmId === ECBF_ALGO_ID) {
    const leaderPanel = document.createElement("div");
    leaderPanel.className = "params-grid";
    const leader = params.leader && typeof params.leader === "object" ? params.leader : { kind: "paper" };
    if (!params.leader || typeof params.leader !== "object") {
      params.leader = leader;
    }
    const ensureLeaderArray = (key) => {
      if (!Array.isArray(leader[key]) || leader[key].length !== 3) {
        leader[key] = [0, 0, 0];
      }
    };
    leaderPanel.append(
      makeSelectField(
        "Leader trajectory",
        [
          { value: "circle", label: "Circle" },
          { value: "paper", label: "Paper (circle + climb)" },
          { value: "static", label: "Static" },
          { value: "poly", label: "Polynomial" },
          { value: "custom", label: "Custom (edit code)" },
        ],
        leader.kind || "paper",
        (v) => {
          leader.kind = v;
          if (v === "circle") {
            ensureLeaderArray("center");
            if (!Number.isFinite(leader.radius)) leader.radius = 6;
            if (!Number.isFinite(leader.omega)) leader.omega = 0.2;
          }
          if (v === "static") ensureLeaderArray("position");
          if (v === "poly") {
            ensureLeaderArray("a2");
            ensureLeaderArray("a1");
            ensureLeaderArray("a0");
          }
          applyAlgorithmParamsToSim(algorithmId);
          renderAlgorithmParamsPanel();
        }
      ),
      makeToggleField("Leader paused", params.leader_paused, (v) => {
        params.leader_paused = v;
        applyAlgorithmParamsToSim(algorithmId);
      })
    );

    if (leader.kind === "static") {
      ensureLeaderArray("position");
      const posPanel = document.createElement("div");
      posPanel.className = "params-grid";
      ["x", "y", "z"].forEach((axis, idx) => {
        posPanel.append(
          makeNumberField(`Leader ${axis}`, leader.position[idx], (v) => {
            leader.position[idx] = Number.isFinite(v) ? v : leader.position[idx];
            applyAlgorithmParamsToSim(algorithmId);
          }, { step: 0.1 })
        );
      });
      algorithmParamsPanel.append(leaderPanel, posPanel);
    } else if (leader.kind === "circle") {
      ensureLeaderArray("center");
      const circlePanel = document.createElement("div");
      circlePanel.className = "params-grid";
      circlePanel.append(
        makeNumberField("Circle radius", leader.radius ?? 6, (v) => {
          leader.radius = Number.isFinite(v) ? v : leader.radius;
          applyAlgorithmParamsToSim(algorithmId);
        }, { step: 0.1, min: 0 })
      );
      algorithmParamsPanel.append(leaderPanel, circlePanel);
    } else if (leader.kind === "poly") {
      ensureLeaderArray("a2");
      ensureLeaderArray("a1");
      ensureLeaderArray("a0");
      const polyPanel = document.createElement("div");
      polyPanel.className = "params-grid";
      const labels = ["x", "y", "z"];
      ["a0", "a1", "a2"].forEach((coeff) => {
        labels.forEach((axis, idx) => {
          polyPanel.append(
            makeNumberField(`Leader ${coeff}.${axis}`, leader[coeff][idx], (v) => {
              leader[coeff][idx] = Number.isFinite(v) ? v : leader[coeff][idx];
              applyAlgorithmParamsToSim(algorithmId);
            }, { step: 0.01 })
          );
        });
      });
      algorithmParamsPanel.append(leaderPanel, polyPanel);
    } else {
      algorithmParamsPanel.append(leaderPanel);
    }
  }
  if (algorithmId === "safe-flocking-alpha") {
    const toggles = document.createElement("div");
    toggles.className = "params-grid";
    toggles.append(
      makeToggleField("Use obstacle CBF", params.use_obstacles, (v) => {
        params.use_obstacles = v;
        applyAlgorithmParamsToSim(algorithmId);
      }),
      makeToggleField("Use inter-agent CBF", params.use_agent_cbf, (v) => {
        params.use_agent_cbf = v;
        applyAlgorithmParamsToSim(algorithmId);
      }),
      makeToggleField("Use moving obstacle terms", params.use_moving_obstacle_terms, (v) => {
        params.use_moving_obstacle_terms = v;
        applyAlgorithmParamsToSim(algorithmId);
      }),
      makeToggleField("2-pass safety filter", params.two_pass, (v) => {
        params.two_pass = v;
        applyAlgorithmParamsToSim(algorithmId);
      })
    );
    algorithmParamsPanel.append(toggles);
  }
  const grid = document.createElement("div");
  grid.className = "params-grid";
  definition.params.forEach((param) => {
    const value = coerceNumber(getParamValue(params, param), 0);
    const field = makeNumberField(
      param.label,
      value,
      (v) => {
        const current = getParamValue(params, param);
        const next = Number.isFinite(v) ? v : current;
        setParamValue(params, param, next);
        if (activeAlgorithmId === algorithmId) {
          applyAlgorithmParamsToSim(algorithmId);
        }
      },
      { step: param.step, min: param.min, max: param.max }
    );
    grid.append(field);
  });
  algorithmParamsPanel.append(grid);
}

function buildRecordMeta(options) {
  const opts = options || {};
  const simRef = opts.simOverride || sim;
  const groupColorObj = {};
  groupColors.forEach((value, key) => {
    groupColorObj[key] = value;
  });
  const modelId =
    opts.modelIdOverride || activeModelId || currentModelId();
  const algorithmId =
    opts.algorithmIdOverride ||
    activeAlgorithmId ||
    currentAlgorithmId(modelId);
  const fields =
    algorithmId === "safe-flocking-alpha"
      ? RECORD_FIELDS_SAFE_FLOCKING
      : RECORD_FIELDS_BASE;
  const plane2d =
    opts.plane2dOverride ??
    (panelPlane2dToggle?.checked ?? plane2dToggle?.checked ?? false);
  const dt =
    simRef && typeof simRef.dt === "function" ? simRef.dt() : currentDt();
  const algorithmParams = ensureAlgorithmParams(algorithmId);
  const algorithmParamsSnapshot = {};
  if (algorithmParams && typeof algorithmParams === "object") {
    for (const [key, value] of Object.entries(algorithmParams)) {
      const num = Number(value);
      if (key && Number.isFinite(num)) {
        algorithmParamsSnapshot[key] = num;
      }
    }
  }
  const meta = {
    version: 1,
    createdAt: new Date().toISOString(),
    dt,
    stride: readRecordStride(),
    maxFrames: readRecordMaxFrames(),
    modelId,
    algorithmId,
    plane2d,
    agentCount: simRef ? simRef.len() : 0,
    fields,
    groupColors: groupColorObj,
  };
  if (Object.keys(algorithmParamsSnapshot).length > 0) {
    meta.algorithmParams = algorithmParamsSnapshot;
  }
  if (simRef && typeof simRef.groups === "function") {
    meta.groups = Array.from(simRef.groups());
  }
  return meta;
}

const recordTextEncoder = new TextEncoder();
const ProtoWire = {
  Varint: 0,
  SixtyFourBit: 1,
  LengthDelimited: 2,
};

function encodeVarint(value) {
  let v = value >>> 0;
  const bytes = [];
  while (v >= 0x80) {
    bytes.push((v & 0x7f) | 0x80);
    v >>>= 7;
  }
  bytes.push(v);
  return new Uint8Array(bytes);
}

function encodeTag(fieldNumber, wireType) {
  return encodeVarint((fieldNumber << 3) | wireType);
}

function concatChunks(chunks, totalLength) {
  const length =
    totalLength ??
    chunks.reduce((sum, chunk) => sum + chunk.length, 0);
  const out = new Uint8Array(length);
  let offset = 0;
  for (const chunk of chunks) {
    out.set(chunk, offset);
    offset += chunk.length;
  }
  return out;
}

function pushUint32Field(fieldNumber, value, chunks) {
  chunks.push(encodeTag(fieldNumber, ProtoWire.Varint));
  chunks.push(encodeVarint(value));
}

function pushBoolField(fieldNumber, value, chunks) {
  chunks.push(encodeTag(fieldNumber, ProtoWire.Varint));
  chunks.push(encodeVarint(value ? 1 : 0));
}

function pushDoubleField(fieldNumber, value, chunks) {
  const buffer = new ArrayBuffer(8);
  new DataView(buffer).setFloat64(0, value, true);
  chunks.push(encodeTag(fieldNumber, ProtoWire.SixtyFourBit));
  chunks.push(new Uint8Array(buffer));
}

function pushStringField(fieldNumber, value, chunks) {
  const bytes = recordTextEncoder.encode(String(value));
  chunks.push(encodeTag(fieldNumber, ProtoWire.LengthDelimited));
  chunks.push(encodeVarint(bytes.length));
  chunks.push(bytes);
}

function pushMessageField(fieldNumber, payload, chunks) {
  if (!payload || payload.length === 0) return;
  chunks.push(encodeTag(fieldNumber, ProtoWire.LengthDelimited));
  chunks.push(encodeVarint(payload.length));
  chunks.push(payload);
}

function pushPackedVarintField(fieldNumber, values, chunks) {
  if (!values || values.length === 0) return;
  const packedChunks = [];
  let packedLength = 0;
  for (const value of values) {
    const bytes = encodeVarint(value);
    packedChunks.push(bytes);
    packedLength += bytes.length;
  }
  if (packedLength === 0) return;
  chunks.push(encodeTag(fieldNumber, ProtoWire.LengthDelimited));
  chunks.push(encodeVarint(packedLength));
  for (const chunk of packedChunks) {
    chunks.push(chunk);
  }
}

function pushPackedFloat32Field(fieldNumber, dataChunks, byteLength, chunks) {
  if (!dataChunks || dataChunks.length === 0 || byteLength === 0) return;
  chunks.push(encodeTag(fieldNumber, ProtoWire.LengthDelimited));
  chunks.push(encodeVarint(byteLength));
  for (const chunk of dataChunks) {
    chunks.push(chunk);
  }
}

function toUint32OrNull(value) {
  const num = Number(value);
  if (!Number.isFinite(num) || num < 0) return null;
  const floored = Math.floor(num);
  return Math.min(0xffffffff, floored) >>> 0;
}

function decodeVarint(data, offset) {
  let value = 0;
  let shift = 0;
  let pos = offset;
  while (pos < data.length) {
    const byte = data[pos];
    pos += 1;
    value |= (byte & 0x7f) << shift;
    if (byte < 0x80) {
      return { value, offset: pos };
    }
    shift += 7;
  }
  throw new Error("Invalid varint");
}

function skipField(data, offset, wireType) {
  if (wireType === 0) {
    return decodeVarint(data, offset).offset;
  }
  if (wireType === 1) {
    return offset + 8;
  }
  if (wireType === 2) {
    const { value: length, offset: next } = decodeVarint(data, offset);
    return next + length;
  }
  if (wireType === 5) {
    return offset + 4;
  }
  throw new Error(`Unsupported wire type ${wireType}`);
}

function decodeString(data, offset) {
  const { value: length, offset: next } = decodeVarint(data, offset);
  const end = next + length;
  const text = new TextDecoder().decode(data.subarray(next, end));
  return { value: text, offset: end };
}

function decodePackedVarints(data, offset) {
  const { value: length, offset: next } = decodeVarint(data, offset);
  const end = next + length;
  const values = [];
  let pos = next;
  while (pos < end) {
    const decoded = decodeVarint(data, pos);
    values.push(decoded.value);
    pos = decoded.offset;
  }
  return { values, offset: end };
}

function decodeAlgorithmParam(data) {
  let offset = 0;
  let key = "";
  let value = Number.NaN;
  while (offset < data.length) {
    const tag = decodeVarint(data, offset);
    const field = tag.value >> 3;
    const wireType = tag.value & 7;
    offset = tag.offset;
    if (field === 1 && wireType === 2) {
      const decoded = decodeString(data, offset);
      key = decoded.value;
      offset = decoded.offset;
    } else if (field === 2 && wireType === 1) {
      value = new DataView(
        data.buffer,
        data.byteOffset + offset,
        8
      ).getFloat64(0, true);
      offset += 8;
    } else {
      offset = skipField(data, offset, wireType);
    }
  }
  return { key, value };
}

function decodeRecordMeta(data) {
  const meta = {
    version: 0,
    createdAt: "",
    dt: Number.NaN,
    stride: 1,
    maxFrames: 0,
    modelId: "",
    algorithmId: "",
    plane2d: false,
    agentCount: 0,
    fields: [],
    algorithmParams: {},
    groups: [],
  };
  let offset = 0;
  while (offset < data.length) {
    const tag = decodeVarint(data, offset);
    const field = tag.value >> 3;
    const wireType = tag.value & 7;
    offset = tag.offset;
    if (field === 1 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      meta.version = decoded.value;
      offset = decoded.offset;
    } else if (field === 2 && wireType === 2) {
      const decoded = decodeString(data, offset);
      meta.createdAt = decoded.value;
      offset = decoded.offset;
    } else if (field === 3 && wireType === 1) {
      meta.dt = new DataView(data.buffer, data.byteOffset + offset, 8).getFloat64(0, true);
      offset += 8;
    } else if (field === 4 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      meta.stride = decoded.value;
      offset = decoded.offset;
    } else if (field === 5 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      meta.maxFrames = decoded.value;
      offset = decoded.offset;
    } else if (field === 6 && wireType === 2) {
      const decoded = decodeString(data, offset);
      meta.modelId = decoded.value;
      offset = decoded.offset;
    } else if (field === 7 && wireType === 2) {
      const decoded = decodeString(data, offset);
      meta.algorithmId = decoded.value;
      offset = decoded.offset;
    } else if (field === 8 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      meta.plane2d = decoded.value !== 0;
      offset = decoded.offset;
    } else if (field === 9 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      meta.agentCount = decoded.value;
      offset = decoded.offset;
    } else if (field === 10 && wireType === 2) {
      const decoded = decodeString(data, offset);
      meta.fields.push(decoded.value);
      offset = decoded.offset;
    } else if (field === 12 && wireType === 2) {
      const decoded = decodePackedVarints(data, offset);
      meta.groups = decoded.values;
      offset = decoded.offset;
    } else if (field === 13 && wireType === 2) {
      const decoded = decodeVarint(data, offset);
      const payload = data.subarray(decoded.offset, decoded.offset + decoded.value);
      const param = decodeAlgorithmParam(payload);
      if (param.key && Number.isFinite(param.value)) {
        meta.algorithmParams[param.key] = param.value;
      }
      offset = decoded.offset + decoded.value;
    } else {
      offset = skipField(data, offset, wireType);
    }
  }
  return meta;
}

function decodeRecording(buffer) {
  const data = new Uint8Array(buffer);
  let metaBytes = null;
  let frameCount = 0;
  let statesBytes = null;
  let offset = 0;
  while (offset < data.length) {
    const tag = decodeVarint(data, offset);
    const field = tag.value >> 3;
    const wireType = tag.value & 7;
    offset = tag.offset;
    if (field === 1 && wireType === 2) {
      const decoded = decodeVarint(data, offset);
      metaBytes = data.subarray(decoded.offset, decoded.offset + decoded.value);
      offset = decoded.offset + decoded.value;
    } else if (field === 2 && wireType === 0) {
      const decoded = decodeVarint(data, offset);
      frameCount = decoded.value;
      offset = decoded.offset;
    } else if (field === 3 && wireType === 2) {
      const decoded = decodeVarint(data, offset);
      statesBytes = data.subarray(decoded.offset, decoded.offset + decoded.value);
      offset = decoded.offset + decoded.value;
    } else {
      offset = skipField(data, offset, wireType);
    }
  }
  if (!statesBytes) {
    throw new Error("Missing states payload");
  }
  if (statesBytes.byteLength % 4 !== 0) {
    throw new Error("Invalid states payload length");
  }
  const meta = metaBytes ? decodeRecordMeta(metaBytes) : {};
  const fields = meta.fields && meta.fields.length > 0 ? meta.fields : defaultStateFields;
  const fieldCount = fields.length;
  let alignedBytes = statesBytes;
  if (alignedBytes.byteOffset % 4 !== 0) {
    alignedBytes = statesBytes.slice();
  }
  const states = new Float32Array(
    alignedBytes.buffer,
    alignedBytes.byteOffset,
    alignedBytes.byteLength / 4
  );
  const totalValues = states.length;
  let agentCount = meta.agentCount || (meta.groups ? meta.groups.length : 0);
  if (!agentCount && frameCount) {
    agentCount = Math.floor(totalValues / (frameCount * fieldCount));
  }
  if (!frameCount && agentCount) {
    frameCount = Math.floor(totalValues / (agentCount * fieldCount));
  }
  if (!frameCount || !agentCount) {
    throw new Error("Unable to infer frame or agent count");
  }
  const expectedValues = frameCount * agentCount * fieldCount;
  if (expectedValues > totalValues) {
    throw new Error("States payload is truncated");
  }
  const trimmedStates = expectedValues === totalValues ? states : states.subarray(0, expectedValues);
  const dt = Number.isFinite(meta.dt) ? meta.dt : 0;
  const stride = meta.stride || 1;
  return {
    meta,
    fields,
    fieldCount,
    frameCount,
    agentCount,
    states: trimmedStates,
    frameStride: agentCount * fieldCount,
    dt,
    stride,
  };
}

function encodeGroupColor(groupId, color) {
  const chunks = [];
  const groupValue = toUint32OrNull(groupId);
  if (groupValue !== null) {
    pushUint32Field(1, groupValue, chunks);
  }
  if (color) {
    pushStringField(2, color, chunks);
  }
  return concatChunks(chunks);
}

function encodeAlgorithmParam(key, value) {
  const chunks = [];
  if (key) {
    pushStringField(1, key, chunks);
  }
  const num = Number(value);
  if (Number.isFinite(num)) {
    pushDoubleField(2, num, chunks);
  }
  return concatChunks(chunks);
}

function encodeRecordMeta(meta) {
  const chunks = [];
  const version = toUint32OrNull(meta?.version) ?? 0;
  if (version !== 0) pushUint32Field(1, version, chunks);
  if (meta?.createdAt) pushStringField(2, meta.createdAt, chunks);
  const dt = Number(meta?.dt);
  if (Number.isFinite(dt)) pushDoubleField(3, dt, chunks);
  const stride = toUint32OrNull(meta?.stride);
  if (stride !== null && stride !== 0) pushUint32Field(4, stride, chunks);
  const maxFrames = toUint32OrNull(meta?.maxFrames);
  if (maxFrames !== null && maxFrames !== 0) {
    pushUint32Field(5, maxFrames, chunks);
  }
  if (meta?.modelId) pushStringField(6, meta.modelId, chunks);
  if (meta?.algorithmId) pushStringField(7, meta.algorithmId, chunks);
  if (meta?.plane2d) pushBoolField(8, true, chunks);
  const agentCount = toUint32OrNull(meta?.agentCount);
  if (agentCount !== null && agentCount !== 0) {
    pushUint32Field(9, agentCount, chunks);
  }
  if (Array.isArray(meta?.fields)) {
    meta.fields.forEach((field) => {
      if (field) pushStringField(10, field, chunks);
    });
  }
  if (meta?.groupColors && typeof meta.groupColors === "object") {
    for (const [group, color] of Object.entries(meta.groupColors)) {
      if (!color) continue;
      const entry = encodeGroupColor(group, String(color));
      if (entry.length > 0) {
        pushMessageField(11, entry, chunks);
      }
    }
  }
  if (Array.isArray(meta?.groups)) {
    const groups = meta.groups
      .map((g) => toUint32OrNull(g))
      .filter((g) => g !== null);
    pushPackedVarintField(12, groups, chunks);
  }
  if (meta?.algorithmParams && typeof meta.algorithmParams === "object") {
    for (const [key, value] of Object.entries(meta.algorithmParams)) {
      if (!key) continue;
      const num = Number(value);
      if (!Number.isFinite(num)) continue;
      const entry = encodeAlgorithmParam(key, num);
      if (entry.length > 0) {
        pushMessageField(13, entry, chunks);
      }
    }
  }
  return concatChunks(chunks);
}

function collectFrameByteChunks(frames) {
  const chunks = [];
  let byteLength = 0;
  for (const frame of frames) {
    const f32 =
      frame instanceof Float32Array ? frame : Float32Array.from(frame);
    const bytes = new Uint8Array(
      f32.buffer,
      f32.byteOffset,
      f32.byteLength
    );
    chunks.push(bytes);
    byteLength += bytes.length;
  }
  return { chunks, byteLength };
}

function encodeRecording(meta, frames) {
  const chunks = [];
  const metaBytes = encodeRecordMeta(meta);
  if (metaBytes.length > 0) {
    pushMessageField(1, metaBytes, chunks);
  }

  const frameCount = toUint32OrNull(frames?.length) ?? 0;
  if (frameCount !== 0) {
    pushUint32Field(2, frameCount, chunks);
  }

  const { chunks: frameChunks, byteLength } = collectFrameByteChunks(frames || []);
  pushPackedFloat32Field(3, frameChunks, byteLength, chunks);

  return concatChunks(chunks);
}

function normalizeFieldName(name) {
  return String(name || "").trim().toLowerCase();
}

function buildPlaybackData(buffer) {
  const decoded = decodeRecording(buffer);
  const fields = decoded.fields.map(normalizeFieldName);
  const xIndex = fields.indexOf("x");
  const yIndex = fields.indexOf("y");
  const zIndex = fields.indexOf("z");
  if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
    throw new Error("Playback data must include x,y,z fields");
  }
  const dt = Number.isFinite(decoded.dt) && decoded.dt > 0 ? decoded.dt : currentDt();
  const stride = decoded.stride > 0 ? decoded.stride : 1;
  return {
    meta: decoded.meta,
    fields,
    frameCount: decoded.frameCount,
    agentCount: decoded.agentCount,
    fieldCount: decoded.fieldCount,
    states: decoded.states,
    frameStride: decoded.frameStride,
    indices: { x: xIndex, y: yIndex, z: zIndex },
    dt,
    stride,
    groups: decoded.meta.groups || [],
  };
}

function playbackTimeStep() {
  if (!playbackData) return 0;
  return playbackData.dt * playbackData.stride;
}

function playbackTimeForIndex(index) {
  return index * playbackTimeStep();
}

function updatePlaybackStatus() {
  if (!playbackActive || !playbackData) {
    if (playbackStatus) playbackStatus.textContent = "No playback";
    if (playbackSlider) {
      playbackSlider.disabled = true;
      playbackSlider.value = "0";
      playbackSlider.max = "0";
      playbackSlider.step = "0.001";
    }
    updatePlaybackControls();
    return;
  }
  const time = playbackTimeForIndex(playbackIndex);
  const algoLabel = playbackData.meta?.algorithmId ? ` • ${playbackData.meta.algorithmId}` : "";
  const planeLabel = playbackData.meta?.plane2d ? " • 2D" : " • 3D";
  if (playbackStatus) {
    playbackStatus.textContent = `Playback ${playbackIndex + 1}/${playbackData.frameCount} • t=${time.toFixed(3)}s${algoLabel}${planeLabel}`;
  }
  if (playbackTimeInput) {
    playbackTimeInput.value = String(time.toFixed(3));
  }
  if (playbackSlider) {
    const maxTime = playbackTimeForIndex(playbackData.frameCount - 1);
    const step = playbackTimeStep();
    playbackSlider.disabled = false;
    playbackSlider.max = String(maxTime);
    playbackSlider.step = String(step > 0 ? step : 0.001);
    playbackSlider.value = String(time);
  }
  updatePlaybackControls();
}

function applyPlaybackFrame() {
  if (!playbackActive || !playbackData) return;
  const { states, frameStride, indices, fieldCount } = playbackData;
  const count = meshes.length;
  const frameOffset = playbackIndex * frameStride;
  for (let i = 0; i < count; i += 1) {
    const base = frameOffset + i * fieldCount;
    const x = states[base + indices.x] ?? 0;
    const y = states[base + indices.y] ?? 0;
    const z = states[base + indices.z] ?? 0;
    meshes[i].position.set(x, y, z);
  }
}

function enterPlayback(buffer) {
  playbackData = buildPlaybackData(buffer);
  playbackActive = true;
  playbackIndex = 0;
  setPaused(true);
  if (recording) setRecording(false);
  rebuildMeshes();
  applyGroupColorsToMeshes();
  renderGroupColors();
  updateAgentTotal();
  updatePlaybackStatus();
}

function clearPlaybackState() {
  playbackActive = false;
  playbackData = null;
  playbackIndex = 0;
  updatePlaybackStatus();
  updateAgentTotal();
}

function exitPlayback() {
  if (!playbackActive) return;
  clearPlaybackState();
  rebuildMeshes();
  applyGroupColorsToMeshes();
  renderGroupColors();
}

function seekPlaybackTime(timeSeconds) {
  if (!playbackActive || !playbackData) return;
  const step = playbackTimeStep();
  if (step <= 0) return;
  const rawIndex = Math.floor(timeSeconds / step);
  playbackIndex = Math.min(Math.max(rawIndex, 0), playbackData.frameCount - 1);
  applyPlaybackFrame();
  updatePlaybackStatus();
}

function advancePlaybackFrame() {
  if (!playbackActive || !playbackData) return;
  if (playbackIndex < playbackData.frameCount - 1) {
    playbackIndex += 1;
  } else {
    setPaused(true);
  }
}

function updateRecordStatus() {
  if (recordStatus) {
    const armed = !recording && !!recordToggle?.checked;
    recordStatus.textContent = recording
      ? `${recordFrames.length} frames • rec`
      : armed
        ? `${recordFrames.length} frames • armed`
        : `${recordFrames.length} frames`;
  }
  if (recordDownloadBtn) {
    recordDownloadBtn.disabled = recordFrames.length === 0;
  }
  if (recordClearBtn) {
    recordClearBtn.disabled = recordFrames.length === 0 && !recording;
  }
}

function setRecording(next) {
  if (recording === next) return;
  recording = next;
  if (recordToggle) recordToggle.checked = recording;
  if (recording) {
    recordFrames = [];
    recordStep = 0;
    recordMeta = buildRecordMeta();
    captureFrame(true);
  }
  updateRecordStatus();
}

function clearRecording() {
  recordFrames = [];
  recordStep = 0;
  if (recording) {
    recordMeta = buildRecordMeta();
    captureFrame(true);
  }
  updateRecordStatus();
}

function captureFrame(force = false) {
  if (!recording || !sim) return;
  const stride = recordMeta?.stride || readRecordStride();
  if (!force && recordStep % stride !== 0) return;
  const algoId = recordMeta?.algorithmId || activeAlgorithmId || currentAlgorithmId(activeModelId || currentModelId());
  const useDebug = algoId === "safe-flocking-alpha" && typeof sim.debug_states === "function";
  const getter = useDebug
    ? sim.debug_states.bind(sim)
    : (typeof sim.states === "function" ? sim.states.bind(sim) : null);
  if (!getter) return;
  const states = getter();
  const frame =
    states instanceof Float32Array ? states : Float32Array.from(states);
  recordFrames.push(frame);
  updateRecordStatus();
  const maxFrames = recordMeta?.maxFrames || 0;
  if (maxFrames > 0 && recordFrames.length >= maxFrames) {
    setRecording(false);
  }
}

function downloadRecording() {
  if (recordFrames.length === 0) return;
  if (!recordMeta) {
    recordMeta = buildRecordMeta();
  }
  const payload = encodeRecording(recordMeta, recordFrames);
  const blob = new Blob([payload], { type: "application/x-protobuf" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `rphys-history-${Date.now()}.pb`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function buildSimForExport(modelId, algorithmId, plane2d) {
  let exportSim = null;
  try {
    if (modelId === customModelId) {
      const cfg = buildCustomConfig(algorithmId);
      exportSim = WasmSim.newFromConfig(cfg);
    } else {
      exportSim = WasmSim.newWithIds(modelId, algorithmId);
    }
  } catch (err) {
    console.error("Failed to create export simulation", err);
    return null;
  }
  if (plane2d && typeof exportSim.set_plane_2d === "function") {
    exportSim.set_plane_2d(true);
  }
  applyAlgorithmParamsToSim(algorithmId, exportSim);
  return exportSim;
}

function setFastExportState(running) {
  fastExportRunning = running;
  if (!recordFastBtn) return;
  recordFastBtn.disabled = running;
  recordFastBtn.textContent = running ? "Exporting..." : "Fast Export (.pb)";
}

function fastExportRecording() {
  if (fastExportRunning) return;
  const targetTime = readFastTime();
  const stride = readRecordStride();
  const maxFrames = readRecordMaxFrames();
  const hasClusters = clusterCountTotal() > 0;
  const liveModel = activeModelId || currentModelId();
  const liveAlgo = activeAlgorithmId || currentAlgorithmId(liveModel);
  const targetModel = sim ? liveModel : (hasClusters ? customModelId : (modelSelect?.value || currentModelId()));
  const targetAlgo = sim ? liveAlgo : (algorithmSelectPanel?.value || algorithmSelect?.value || currentAlgorithmId(targetModel));
  const plane2dDesired = panelPlane2dToggle?.checked ?? plane2dToggle?.checked ?? false;

  const exportSim = buildSimForExport(targetModel, targetAlgo, plane2dDesired);
  if (!exportSim) return;
  const useDebug =
    targetAlgo === "safe-flocking-alpha" &&
    typeof exportSim.debug_states === "function";
  const getFrame = useDebug
    ? exportSim.debug_states.bind(exportSim)
    : exportSim.states.bind(exportSim);
  const dt = typeof exportSim.dt === "function" ? exportSim.dt() : 0;
  const totalSteps = dt > 0 ? Math.ceil(targetTime / dt) : 0;

  setFastExportState(true);
  try {
    const frames = [];
    const initialStates = getFrame();
    frames.push(
      initialStates instanceof Float32Array ? initialStates : Float32Array.from(initialStates)
    );
    if (maxFrames === 0 || frames.length < maxFrames) {
      for (let step = 1; step <= totalSteps; step += 1) {
        exportSim.tick();
        if (step % stride === 0) {
          const states = getFrame();
          frames.push(states instanceof Float32Array ? states : Float32Array.from(states));
          if (maxFrames > 0 && frames.length >= maxFrames) {
            break;
          }
        }
      }
    }

    const meta = buildRecordMeta({
      simOverride: exportSim,
      modelIdOverride: targetModel,
      algorithmIdOverride: targetAlgo,
      plane2dOverride: plane2dDesired,
    });
    const payload = encodeRecording(meta, frames);
    const blob = new Blob([payload], { type: "application/x-protobuf" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `rphys-history-fast-${Date.now()}.pb`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
  } finally {
    setFastExportState(false);
  }
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
    algorithmParams: buildAlgorithmParamsConfig(),
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

  if (cfg.algorithmParams && typeof cfg.algorithmParams === "object") {
    loadAlgorithmParamsConfig(cfg.algorithmParams);
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
  renderAlgorithmParamsPanel();

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
  if (playbackActive) {
    clearPlaybackState();
  }
  if (recording) {
    setRecording(false);
  }
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
  simTime = 0;
  leaderHoldState = null;
  if (plane2dToggle) {
    sim.set_plane_2d(plane2dToggle.checked);
  }
  applyAlgorithmParamsToSim(actualAlgo);
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

let activeSetupTab = "initial";

function setSetupTab(tabId) {
  activeSetupTab = tabId;
  setupTabButtons.forEach((btn) => {
    btn.classList.toggle("is-active", btn.dataset.setupTab === tabId);
  });
  setupTabContents.forEach((panel) => {
    panel.classList.toggle("is-active", panel.dataset.setupContent === tabId);
  });
  if (tabId === "parameters") {
    renderAlgorithmParamsPanel();
  }
}

if (setupTabButtons.length > 0) {
  setupTabButtons.forEach((btn) => {
    btn.addEventListener("click", () => {
      const tabId = btn.dataset.setupTab || "initial";
      setSetupTab(tabId);
    });
  });
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
    const recordAfterApply = !!recordToggle?.checked;

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
    if (recordToggle) {
      setRecording(recordAfterApply);
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

if (recordToggle) {
  recordToggle.addEventListener("change", () => {
    updateRecordStatus();
  });
}

if (recordStrideInput) {
  recordStrideInput.addEventListener("input", () => {
    const stride = readRecordStride();
    recordStrideInput.value = String(stride);
    if (recording && recordMeta) recordMeta.stride = stride;
  });
}

if (recordMaxFramesInput) {
  recordMaxFramesInput.addEventListener("input", () => {
    const maxFrames = readRecordMaxFrames();
    recordMaxFramesInput.value = String(maxFrames);
    if (recording && recordMeta) recordMeta.maxFrames = maxFrames;
  });
}

if (recordFastTimeInput) {
  recordFastTimeInput.addEventListener("input", () => {
    const time = readFastTime();
    recordFastTimeInput.value = String(time);
  });
}

if (recordDownloadBtn) {
  recordDownloadBtn.addEventListener("click", () => downloadRecording());
}

if (recordClearBtn) {
  recordClearBtn.addEventListener("click", () => clearRecording());
}

if (recordFastBtn) {
  recordFastBtn.addEventListener("click", () => fastExportRecording());
}

if (playbackLoadBtn && playbackFileInput) {
  playbackLoadBtn.addEventListener("click", () => {
    playbackFileInput.value = "";
    playbackFileInput.click();
  });

  playbackFileInput.addEventListener("change", () => {
    const file = playbackFileInput.files && playbackFileInput.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        enterPlayback(reader.result);
      } catch (err) {
        console.error("Failed to load playback", err);
        clearPlaybackState();
      }
    };
    reader.readAsArrayBuffer(file);
  });
}

if (playbackExitBtn) {
  playbackExitBtn.addEventListener("click", () => exitPlayback());
}

if (playbackSeekBtn) {
  playbackSeekBtn.addEventListener("click", () => {
    const time = Number(playbackTimeInput?.value) || 0;
    setPaused(true);
    seekPlaybackTime(time);
  });
}

if (playbackGoBtn) {
  playbackGoBtn.addEventListener("click", () => {
    if (!playbackActive) return;
    setPaused(!paused);
  });
}

if (playbackSlider) {
  playbackSlider.addEventListener("input", () => {
    if (!playbackActive) return;
    const time = Number(playbackSlider.value) || 0;
    setPaused(true);
    seekPlaybackTime(time);
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
renderAlgorithmParamsPanel();
setSetupTab(activeSetupTab);
updateRecordStatus();

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
  if (playbackActive) return;
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

function updateAttitudeHelpers() {
  const show =
    !playbackActive &&
    sim &&
    activeAlgorithmId === ECBF_ALGO_ID &&
    typeof sim.attitudes === "function";
  attitudeGroup.visible = !!show;
  if (!show) return;
  const data = sim.attitudes();
  const count = Math.min(attitudeHelpers.length, Math.floor(data.length / 3));
  for (let i = 0; i < count; i += 1) {
    const base = i * 3;
    const phi = data[base + 0] || 0;
    const theta = data[base + 1] || 0;
    const psi = data[base + 2] || 0;
    const helper = attitudeHelpers[i];
    helper.visible = true;
    helper.position.copy(meshes[i].position);
    helper.rotation.set(phi, theta, psi, "XYZ");
  }
  for (let i = count; i < attitudeHelpers.length; i += 1) {
    attitudeHelpers[i].visible = false;
  }
}

function leaderStateAt(leader, t) {
  if (!leader || typeof leader !== "object") return null;
  const kind = leader.kind || "paper";
  if (kind === "static") {
    const pos = leader.position || [0, 0, 0];
    return { pos: new THREE.Vector3(pos[0], pos[1], pos[2]) };
  }
  if (kind === "circle") {
    const center = leader.center || [0, 0, 0];
    const radius = Number(leader.radius ?? 6);
    const omega = Number(leader.omega ?? 0.2);
    const angle = omega * t;
    return {
      pos: new THREE.Vector3(
        center[0] + radius * Math.cos(angle),
        center[1] + radius * Math.sin(angle),
        center[2] ?? 0
      ),
    };
  }
  if (kind === "poly") {
    const a2 = leader.a2 || [0, 0, 0];
    const a1 = leader.a1 || [0, 0, 0];
    const a0 = leader.a0 || [0, 0, 0];
    return {
      pos: new THREE.Vector3(
        a2[0] * t * t + a1[0] * t + a0[0],
        a2[1] * t * t + a1[1] * t + a0[1],
        a2[2] * t * t + a1[2] * t + a0[2]
      ),
    };
  }
  if (kind === "paper") {
    const omega = -0.06;
    const phase = Math.PI;
    const angle = omega * t + phase;
    return {
      pos: new THREE.Vector3(
        60 + 25 * Math.cos(angle),
        60 + 25 * Math.sin(angle),
        0.5 * t
      ),
    };
  }
  return null;
}

function buildLeaderPath(leader, t) {
  const kind = leader?.kind || "paper";
  const points = [];
  if (kind === "static") return points;
  if (kind === "circle") {
    const center = leader.center || [0, 0, 0];
    const radius = Number(leader.radius ?? 6);
    const segments = 120;
    for (let i = 0; i <= segments; i += 1) {
      const angle = (i / segments) * Math.PI * 2;
      points.push(
        new THREE.Vector3(
          center[0] + radius * Math.cos(angle),
          center[1] + radius * Math.sin(angle),
          center[2] ?? 0
        )
      );
    }
    return points;
  }
  const span = kind === "paper" ? 50 : 30;
  const steps = 160;
  const t0 = t - span * 0.5;
  const dt = span / steps;
  for (let i = 0; i <= steps; i += 1) {
    const ti = t0 + dt * i;
    const state = leaderStateAt(leader, ti);
    if (state?.pos) points.push(state.pos);
  }
  return points;
}

function updateLeaderVisual() {
  const show =
    !playbackActive &&
    sim &&
    activeAlgorithmId === ECBF_ALGO_ID &&
    typeof sim.positions === "function";
  leaderMarker.visible = !!show;
  leaderPathLine.visible = !!show;
  if (!show) {
    leaderHoldState = null;
    return;
  }
  const params = ensureAlgorithmParams(ECBF_ALGO_ID);
  const leader = params.leader || { kind: "paper" };
  const timeScale = Number(params.leader_time_scale ?? 1) || 1;
  const paused = !!params.leader_paused;
  const t = simTime * timeScale;
  let state = leaderStateAt(leader, t);
  if (paused) {
    if (!leaderHoldState) leaderHoldState = state;
    state = leaderHoldState;
  } else {
    leaderHoldState = null;
  }
  if (!state?.pos) {
    leaderMarker.visible = false;
    leaderPathLine.visible = false;
    return;
  }
  leaderMarker.position.copy(state.pos);

  const signature = JSON.stringify({
    kind: leader.kind,
    center: leader.center,
    radius: leader.radius,
    omega: leader.omega,
    a0: leader.a0,
    a1: leader.a1,
    a2: leader.a2,
    position: leader.position,
  });
  const timeVarying = leader.kind === "paper" || leader.kind === "poly";
  const rebuild =
    signature !== leaderPathSignature ||
    (!paused && timeVarying && Math.abs(simTime - leaderPathTime) > 1.5);
  if (rebuild) {
    const points = buildLeaderPath(leader, t);
    if (points.length > 0) {
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      leaderPathLine.geometry.dispose();
      leaderPathLine.geometry = geometry;
      leaderPathLine.visible = true;
    } else {
      leaderPathLine.visible = false;
    }
    leaderPathSignature = signature;
    leaderPathTime = simTime;
  }
}

function animate() {
  if (!sim && !playbackActive) {
    requestAnimationFrame(animate);
    return;
  }
  if (!playbackActive && sim && dragging && dragIndex !== null) {
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
    if (playbackActive) {
      advancePlaybackFrame();
    } else if (sim) {
      sim.tick();
      const step = typeof sim.dt === "function" ? sim.dt() : currentDt();
      if (Number.isFinite(step)) simTime += step;
      if (recording) {
        recordStep += 1;
        captureFrame();
      }
    }
  }

  if (!playbackActive && sim && dragging && dragIndex !== null) {
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

  if (playbackActive) {
    applyPlaybackFrame();
    updatePlaybackStatus();
  } else if (sim) {
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
  }
  updateAttitudeHelpers();
  updateLeaderVisual();

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
