import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import init, { WasmSim } from "./pkg/rphys.js";

await init();

const viewport = document.getElementById("viewport");
const homeButton = document.getElementById("home-btn");
const pauseButton = document.getElementById("sim-toggle-btn");
const restartButton = document.getElementById("sim-restart-btn");

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.setSize(viewport.clientWidth, viewport.clientHeight, false);
viewport.appendChild(renderer.domElement);

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

const gizmoScene = new THREE.Scene();
gizmoScene.background = new THREE.Color(0x11161b);
const gizmoCamera = new THREE.PerspectiveCamera(35, 1, 0.1, 10);
gizmoCamera.position.set(0, 0, 3);
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
gizmoRoot.add(gizmoCube, gizmoEdges, gizmoAxes);
gizmoScene.add(gizmoRoot);

const gizmoViewportSize = new THREE.Vector2();
const gizmoSize = 96;
const gizmoMargin = 14;
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

let sim = WasmSim.new_demo();
const count = sim.len();
let paused = false;

function setPaused(next) {
  paused = next;
  if (!pauseButton) return;
  pauseButton.dataset.state = paused ? "paused" : "running";
  pauseButton.textContent = paused ? "Resume" : "Pause";
  pauseButton.setAttribute("aria-pressed", paused ? "true" : "false");
}

function resetSimulation() {
  sim = WasmSim.new_demo();
  if (plane2dToggle) {
    sim.set_plane_2d(plane2dToggle.checked);
  }
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
  });
}

const geometry = new THREE.SphereGeometry(0.12, 16, 16);
const material = new THREE.MeshStandardMaterial({
  color: 0x6ad1ff,
  roughness: 0.35,
  metalness: 0.1,
});

const meshes = [];
for (let i = 0; i < count; i += 1) {
  const mesh = new THREE.Mesh(geometry, material);
  mesh.userData.index = i;
  scene.add(mesh);
  meshes.push(mesh);
}

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

function renderGizmo(width, height) {
  const inset = gizmoSize;
  const margin = gizmoMargin;
  const x = width - inset - margin;
  const y = height - inset - margin;
  if (x < 0 || y < 0) return;

  renderer.setScissorTest(true);
  renderer.setScissor(x, y, inset, inset);
  renderer.setViewport(x, y, inset, inset);
  renderer.clearDepth();

  gizmoRoot.quaternion.copy(camera.quaternion).invert();

  renderer.render(gizmoScene, gizmoCamera);
  renderer.setScissorTest(false);
}

function animate() {
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
  renderer.render(scene, camera);
  renderGizmo(width, height);
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
}

window.addEventListener("resize", resizeRenderer);
resizeRenderer();
