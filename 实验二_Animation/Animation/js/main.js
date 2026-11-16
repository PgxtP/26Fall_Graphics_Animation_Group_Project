import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { loadRobotAsset } from './robot-model.js';
import './workspace-ui.js';

const viewport = document.getElementById('viewport');
const status = document.getElementById('scene-status');

try {
  initScene();
} catch (error) {
  status.textContent = `场景初始化失败：${error.message}`;
  console.error(error);
}

function initScene() {
  // 场景与相机：采用 three.js 默认的 Y 轴向上约定。
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101822);
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  camera.position.set(5, 4, 6);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.setAttribute('aria-label', '老师提供的关节化机械臂模型');
  viewport.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.6, 0);
  controls.enableDamping = true;
  controls.minDistance = 2;
  controls.maxDistance = 20;
  controls.maxPolarAngle = Math.PI / 2 - 0.03;
  controls.update();
  controls.saveState();

  scene.add(new THREE.HemisphereLight(0xd9efff, 0x485362, 2));
  const light = new THREE.DirectionalLight(0xffffff, 3);
  light.position.set(4, 7, 4);
  light.castShadow = true;
  light.shadow.mapSize.set(1024, 1024);
  light.shadow.camera.left = -6;
  light.shadow.camera.right = 6;
  light.shadow.camera.top = 6;
  light.shadow.camera.bottom = -6;
  light.shadow.normalBias = 0.03;
  scene.add(light);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(20, 20),
    new THREE.MeshStandardMaterial({ color: 0x1b2938, roughness: 0.9 }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  const grid = new THREE.GridHelper(20, 20, 0x648294, 0x334858);
  grid.position.y = 0.005;
  scene.add(grid);
  const axes = new THREE.AxesHelper(2.5);
  axes.position.y = 0.015;
  scene.add(axes);

  let simulation;
  loadRobotAsset({ scene, camera, controls, floor, grid, axes, light, viewport })
    .then((loaded) => { simulation = loaded; })
    .catch((error) => {
      status.textContent = `模型加载失败：${error.message}。请检查 Network 和 Console。`;
      console.error(error);
    });
  document.getElementById('reset-view').addEventListener('click', () => controls.reset());

  // 根据场景容器尺寸调整画布与相机，避免页面缩放后拉伸。
  function resize() {
    const width = Math.max(viewport.clientWidth, 1);
    const height = Math.max(viewport.clientHeight, 1);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    renderer.render(scene, camera);
  }
  new ResizeObserver(resize).observe(viewport);
  resize();

  let previousTime;
  renderer.setAnimationLoop((time) => {
    const delta = previousTime === undefined ? 0 : (time - previousTime) / 1000;
    previousTime = time;
    simulation?.update(delta);
    controls.update();
    renderer.render(scene, camera);
  });

  renderer.domElement.addEventListener('webglcontextlost', (event) => {
    event.preventDefault();
    renderer.setAnimationLoop(null);
    status.textContent = '图形上下文已丢失，请刷新页面重新加载。';
  });
  const threeVer = document.getElementById('three-version');
  if (threeVer) threeVer.textContent = `r${THREE.REVISION} / 0.186.1`;
  status.textContent = '场景已启动，正在读取模型与关节配置…';
  console.info('three.js 场景已启动', { revision: THREE.REVISION, renderer, scene, camera });
}
