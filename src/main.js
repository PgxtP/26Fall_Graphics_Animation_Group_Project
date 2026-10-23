import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RobotModel } from './robot/RobotModel.js';
import { Kinematics } from './robot/Kinematics.js';
import { ROBOT_CONFIG } from './robot/robot3Config.js';
import { verifyRobotFK } from './robot/verifyRobotFK.js';
import './style.css';

const $ = selector => document.querySelector(selector);
const status = $('#status');
const errorStatus = message => { status.textContent = message; status.className = 'status error'; };
const degrees = THREE.MathUtils.radToDeg;
const radians = THREE.MathUtils.degToRad;

// A/B/C/D 交付后在此导入并实例化，再调用 connectTeamModules；不把骨架视作已集成。
async function start() {
  const canvas = $('#scene');
  const viewport = $('#viewport');
  const context = canvas.getContext('webgl2', { antialias: true });
  if (!context) throw new Error('WebGL2 不可用，请开启浏览器硬件加速');
  const renderer = new THREE.WebGLRenderer({ canvas, context, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#e9edf0');
  const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 1000);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x66717d, 2.7));
  const key = new THREE.DirectionalLight(0xffffff, 3.6);
  key.position.set(18, 28, 18);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xc2deff, 1.5);
  fill.position.set(-18, 10, -10);
  scene.add(fill);

  const gltf = await new GLTFLoader().loadAsync(import.meta.env.BASE_URL + 'models/robot3/robot_articulated.glb');
  const placement = new THREE.Group();
  placement.position.fromArray(ROBOT_CONFIG.placement.position);
  placement.quaternion.fromArray(ROBOT_CONFIG.placement.quaternion);
  placement.scale.fromArray(ROBOT_CONFIG.placement.scale);
  placement.add(gltf.scene);
  scene.add(placement);
  const robot = new RobotModel(gltf.scene);
  const kinematics = new Kinematics(robot.getKinematicConfig());
  const bounds = new THREE.Box3().setFromObject(gltf.scene);
  const span = bounds.getSize(new THREE.Vector3()).length();
  const grid = new THREE.GridHelper(span * 3, 60, 0xadb7bf, 0xd1d9de);
  grid.position.y = -0.03;
  scene.add(grid);
  const baseAxes = new THREE.AxesHelper(span * 0.17);
  baseAxes.position.y = 0.02;
  scene.add(baseAxes);
  const flangeAxes = new THREE.AxesHelper(span * 0.085);
  robot.flange.add(flangeAxes);
  const tcpMarker = new THREE.Mesh(new THREE.SphereGeometry(span * 0.008, 16, 12), new THREE.MeshBasicMaterial({ color: 0xe87429 }));
  scene.add(tcpMarker);
  const target = bounds.getCenter(new THREE.Vector3());
  function fitCamera() {
    controls.target.copy(target);
    const distance = span / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)));
    camera.position.copy(target).add(new THREE.Vector3(1, 0.65, 1).normalize().multiplyScalar(distance * 1.18));
    camera.near = span / 500;
    camera.far = span * 100;
    camera.updateProjectionMatrix();
    controls.update();
  }
  function resize() {
    const { width, height } = viewport.getBoundingClientRect();
    renderer.setSize(width, height, false);
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(viewport);
  resize();
  fitCamera();

  let state = 'ENABLED';
  let toolOffset;
  let demoTime = 0;
  let demo = false;
  let demoOrigin = robot.getJointAngles();
  let latestVerification;
  const team = {};
  let elapsed = 0;
  let lastSampleTime = 0;
  let lastSamplePosition;
  const debugRows = [];

  const jointContainer = $('#joint-controls');
  robot.jointLimits.forEach((limit, i) => {
    const row = document.createElement('label');
    row.className = 'joint-row';
    row.innerHTML = '<span class="joint-name">J' + (i + 1) + '</span>' +
      '<input type="range" aria-label="J' + (i + 1) + ' 角度滑块" min="' + degrees(limit.min).toFixed(6) + '" max="' + degrees(limit.max).toFixed(6) + '" step="0.1" value="0">' +
      '<input type="number" aria-label="J' + (i + 1) + ' 角度数值" min="' + degrees(limit.min).toFixed(6) + '" max="' + degrees(limit.max).toFixed(6) + '" step="0.1" value="0"><span>°</span>';
    const range = row.querySelector('[type=range]');
    const number = row.querySelector('[type=number]');
    function commit(value) {
      if (!Number.isFinite(value)) { updateReadout(); return; }
      const angles = robot.getJointAngles();
      angles[i] = radians(value);
      applyAngles(angles);
      number.value = degrees(robot.getJointAngles()[i]).toFixed(1);
    }
    range.addEventListener('input', () => commit(range.valueAsNumber));
    number.addEventListener('change', () => commit(number.valueAsNumber));
    jointContainer.append(row);
    debugRows.push({ range, number });
  });

  function updateReadout() {
    const angles = robot.getJointAngles();
    debugRows.forEach((row, i) => { row.range.value = degrees(angles[i]); if (document.activeElement !== row.number) row.number.value = degrees(angles[i]).toFixed(1); });
    const pose = robot.getTCPPose(toolOffset);
    tcpMarker.position.copy(pose.position);
    ['x', 'y', 'z'].forEach(axis => { $('#tcp-' + axis).textContent = pose.position[axis].toFixed(4); });
    $('#tcp-rpy').textContent = Object.values(pose.rotationRPY).map(a => degrees(a).toFixed(2)).join(' / ') + ' °';
    $('#tcp-quaternion').textContent = pose.quaternion.toArray().map(a => a.toFixed(4)).join(' / ');
    const verification = kinematics.verifyFK(angles, pose.matrixWorld, toolOffset);
    $('#live-error').textContent = verification.matrixMaxDiff.toExponential(2);
    $('#live-error').className = verification.isPassed ? 'pass' : 'fail';
    team.teachingBox?.updateTCPDisplay(pose);
    return pose;
  }

  function stopMotion() {
    demo = false;
    $('#demo').textContent = '关节联动演示';
    team.planner?.stop();
  }

  function applyAngles(angles) {
    if (state !== 'ENABLED') return { success: false, clampedAngles: robot.getJointAngles(), errorMsg: '请先解除急停并使能' };
    stopMotion();
    const result = robot.setJointAngles(angles);
    if (result.success) {
      team.teachingBox?.syncSliders(result.clampedAngles);
      updateReadout();
    }
    $('#feedback').textContent = result.errorMsg ?? '关节已更新 · 内部单位 rad';
    return result;
  }

  function setState(next) {
    if (!['ENABLED', 'DISABLED', 'ESTOP'].includes(next)) throw new TypeError('未知控制状态');
    // 急停只能显式复位；A 的滑块/使能事件不能直接清除急停。
    if (state === 'ESTOP' && next !== 'ESTOP') return;
    state = next;
    if (state !== 'ENABLED') stopMotion();
    refreshState();
  }

  function refreshState() {
    $('#state').textContent = { ENABLED: '已使能', DISABLED: '已失能', ESTOP: '急停已锁定' }[state];
    $('#state').dataset.state = state;
    $('#enable').textContent = state === 'ENABLED' ? '失能' : '使能';
    $('#enable').disabled = state === 'ESTOP';
    $('#reset-estop').hidden = state !== 'ESTOP';
    $('#zero').disabled = state !== 'ENABLED';
    $('#demo').disabled = state !== 'ENABLED';
    debugRows.forEach(({ range, number }) => { range.disabled = number.disabled = state !== 'ENABLED'; });
  }

  $('#zero').addEventListener('click', () => applyAngles([0, 0, 0, 0, 0, 0]));
  $('#fit').addEventListener('click', fitCamera);
  $('#axes').addEventListener('change', event => { baseAxes.visible = flangeAxes.visible = event.target.checked; });
  $('#enable').addEventListener('click', () => setState(state === 'ENABLED' ? 'DISABLED' : 'ENABLED'));
  $('#estop').addEventListener('click', () => setState('ESTOP'));
  $('#reset-estop').addEventListener('click', () => { state = 'DISABLED'; refreshState(); });
  $('#demo').addEventListener('click', () => {
    if (demo) { stopMotion(); return; }
    if (state !== 'ENABLED') return;
    team.planner?.stop();
    demoOrigin = robot.getJointAngles();
    demoTime = 0;
    demo = true;
    $('#demo').textContent = '停止联动演示';
  });

  function runVerification() {
    latestVerification = verifyRobotFK(robot, kinematics, toolOffset);
    $('#fk-summary').textContent = latestVerification.passed + ' / ' + latestVerification.total + ' 通过';
    $('#fk-summary').className = latestVerification.passed === latestVerification.total ? 'pass' : 'fail';
    $('#fk-max').textContent = latestVerification.maxMatrixDiff.toExponential(3);
    $('#fk-position').textContent = latestVerification.maxPositionError.toExponential(3) + ' m';
    updateReadout();
    return latestVerification;
  }
  $('#verify').addEventListener('click', runVerification);
  $('#export-fk').addEventListener('click', () => {
    const data = runVerification();
    const url = URL.createObjectURL(new Blob([JSON.stringify({ ...data, source: ROBOT_CONFIG.source, sha256: ROBOT_CONFIG.sha256, unitNote: ROBOT_CONFIG.unitNote }, null, 2)], { type: 'application/json' }));
    const link = document.createElement('a');
    link.href = url; link.download = 'leader-fk-verification.json'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });

  /** 接收已经实例化的组员模块，返回联调上下文；允许先接入已交付的部分。 */
  function connectTeamModules(modules) {
    const methods = {
      teachingBox: ['onJointChange', 'onStateChange', 'syncSliders', 'updateTCPDisplay', 'getSpeedRatio'],
      planner: ['update', 'stop'], charts: ['recordSample'], poseManager: ['savePose', 'getPoseList'],
    };
    for (const [name, required] of Object.entries(methods)) {
      if (modules[name] && required.some(method => typeof modules[name][method] !== 'function')) throw new TypeError(name + ' 接口不完整');
    }
    stopMotion();
    Object.assign(team, modules);
    if (modules.teachingBox) {
      modules.teachingBox.onJointChange(applyAngles);
      modules.teachingBox.onStateChange(setState);
    }
    lastSampleTime = elapsed;
    lastSamplePosition = undefined;
    // “已接入实例”仅表示完成绑定，功能正确性仍需该组员模块的联调验收。
    Object.entries({ A: 'teachingBox', B: 'workstation', C: 'planner', D: 'charts' }).forEach(([role, key]) => {
      $('#module-' + role).textContent = team[key] ? '已接入实例 · 待联调' : '待组员交付';
    });
    updateReadout();
    return window.robotApp;
  }

  const timer = new THREE.Timer();
  timer.connect(document);
  function animate(timestamp) {
    timer.update(timestamp);
    const dt = Math.min(timer.getDelta(), 0.1);
    elapsed += dt;
    try {
      if (state === 'ENABLED' && demo) {
        demoTime += dt;
        // 组长调试用周期运动，不是 C 的 MOVJ 程序；从当前姿态平滑开始。
        const ramp = 1 - Math.exp(-2 * demoTime);
        const angles = demoOrigin.map((angle, i) => angle + radians(18) * ramp * Math.sin(demoTime * 0.7 + i * 0.4) * Math.sin(demoTime * 0.5));
        robot.setJointAngles(angles);
        team.teachingBox?.syncSliders(robot.getJointAngles());
      } else if (state === 'ENABLED' && team.planner?.isMoving) {
        // update 返回最终帧时也必须写入，不能先按返回的 isMoving=false 提前跳过。
        const frame = team.planner.update(dt);
        const result = robot.setJointAngles(frame.currentAngles);
        if (!result.success) throw new Error(result.errorMsg);
        team.teachingBox?.syncSliders(result.clampedAngles);
      }
      const pose = updateReadout();
      if (team.charts && elapsed - lastSampleTime >= 0.02) {
        const sampleDt = elapsed - lastSampleTime;
        const velocity = lastSamplePosition ? pose.position.distanceTo(lastSamplePosition) / sampleDt : 0;
        team.charts.recordSample(elapsed, pose.position.clone(), velocity);
        lastSampleTime = elapsed;
        lastSamplePosition = pose.position.clone();
      }
    } catch (error) {
      stopMotion();
      state = 'DISABLED';
      refreshState();
      errorStatus('模块运行失败：' + error.message);
      console.error(error);
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  window.robotApp = {
    robot, kinematics, scene, camera, renderer, controls, jointLimits: robot.jointLimits,
    connectTeamModules, setJointAngles: applyAngles,
    fkPosition: angles => new THREE.Vector3().setFromMatrixPosition(kinematics.forwardKinematics(angles, toolOffset)),
    setToolOffset(matrix) { robot.getTCPPose(matrix); toolOffset = matrix?.clone(); updateReadout(); },
    getTCPPose: () => robot.getTCPPose(toolOffset),
    runVerification, getState: () => state,
    // 为 B 提供法兰引用，为 A/C 提供共享模块上下文。
    flange: robot.flange,
  };
  updateReadout();
  refreshState();
  runVerification();
  status.textContent = '模型已就绪 · 六关节可控 · WebGL2';
  status.className = 'status ready';
  $('#model-loading').hidden = true;
  requestAnimationFrame(animate);
}
start().catch(error => {
  errorStatus('初始化失败：' + error.message);
  $('#model-loading').textContent = '加载失败，请查看上方状态提示';
  console.error(error);
});
