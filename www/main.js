import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import init, { WasmSim } from "./pkg/nginphy.js";

await init();

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);

const camera = new THREE.PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  0.1,
  100
);
camera.position.set(0, 0, 12);

const ambient = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambient);
const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
keyLight.position.set(6, 10, 8);
scene.add(keyLight);

const sim = WasmSim.new_demo();
const count = sim.len();

const geometry = new THREE.SphereGeometry(0.12, 16, 16);
const material = new THREE.MeshStandardMaterial({
  color: 0x6ad1ff,
  roughness: 0.35,
  metalness: 0.1,
});

const meshes = [];
for (let i = 0; i < count; i += 1) {
  const mesh = new THREE.Mesh(geometry, material);
  scene.add(mesh);
  meshes.push(mesh);
}

function animate() {
  sim.tick();
  const positions = sim.positions();
  for (let i = 0; i < count; i += 1) {
    const base = i * 3;
    meshes[i].position.set(
      positions[base + 0],
      positions[base + 1],
      positions[base + 2]
    );
  }

  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

animate();

window.addEventListener("resize", () => {
  const { innerWidth, innerHeight } = window;
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
