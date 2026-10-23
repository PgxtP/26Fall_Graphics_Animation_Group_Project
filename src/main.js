import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import './style.css';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const loader = new GLTFLoader();
const gltf = await loader.loadAsync('/models/robot3/robot_articulated.glb');


const canvas = document.querySelector('#scene');
const status = document.querySelector('#status');
const context = canvas.getContext('webgl2', { antialias: true });

if (!context) {
  status.textContent = 'WebGL2 不可用，请检查浏览器硬件加速设置';
  status.classList.add('error');
  throw new Error('WebGL2 is unavailable');
}

const renderer = new THREE.WebGLRenderer({ canvas, context, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x07101f);

const camera = new THREE.PerspectiveCamera(
  50,
  window.innerWidth / window.innerHeight,
  0.1,
  100,
);
camera.position.set(5.5, 3.2, 7.5);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.2, 0);

scene.add(new THREE.HemisphereLight(0x9ecbff, 0x172033, 1.8));

const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
keyLight.position.set(4, 6, 3);
scene.add(keyLight);

const planet = new THREE.Mesh(
  new THREE.SphereGeometry(1.45, 64, 32),
  new THREE.MeshStandardMaterial({
    color: 0x4f86f7,
    roughness: 0.55,
    metalness: 0.08,
  }),
);
scene.add(planet);

const wireframe = new THREE.Mesh(
  new THREE.SphereGeometry(1.49, 24, 16),
  new THREE.MeshBasicMaterial({
    color: 0xaed0ff,
    wireframe: true,
    transparent: true,
    opacity: 0.22,
  }),
);
scene.add(wireframe);

const orbit = new THREE.Mesh(
  new THREE.RingGeometry(2.7, 2.72, 128),
  new THREE.MeshBasicMaterial({
    color: 0x5679ad,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.55,
  }),
);
orbit.rotation.x = -Math.PI / 2;
scene.add(orbit);

const satellite = new THREE.Mesh(
  new THREE.SphereGeometry(0.28, 32, 16),
  new THREE.MeshStandardMaterial({ color: 0xf6c85f, roughness: 0.7 }),
);
scene.add(satellite);

const version = context.getParameter(context.VERSION);
status.textContent = `机械臂模型加载成功 · WebGL2 可用：${version}`;
status.classList.add('ready');

const timer = new THREE.Timer();
timer.connect(document);
let orbitAngle = 0;

function animate(timestamp) {
  timer.update(timestamp);
  const dt = Math.min(timer.getDelta(), 0.1);
  orbitAngle = (orbitAngle + 0.65 * dt) % (Math.PI * 2);

  planet.rotation.y += 0.35 * dt;
  wireframe.rotation.y -= 0.18 * dt;
  satellite.position.set(
    Math.cos(orbitAngle) * 2.7,
    0.25 * Math.sin(orbitAngle * 2),
    Math.sin(orbitAngle) * 2.7,
  );

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

function resize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

window.addEventListener('resize', resize);
requestAnimationFrame(animate);
