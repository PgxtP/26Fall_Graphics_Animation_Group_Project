/**
 * 【王静】analysis-panel —— 「轨迹分析」「可达区域」两个页签的接线与界面逻辑
 *
 * 只做“把两个模块接到页面上”这一件事，不修改任何队友的代码：
 *   · 轨迹分析：逐帧读取真实 tcp 节点的世界坐标 → TrajectoryCharts 绘制并导出
 *   · 可达区域：按真实关节限位随机采样，借场景树正运动学求 TCP → WorkspaceCloud 统计
 *
 * 依赖：window.robotSimulation（由 robot-model.js 暴露）、urdf-model.js 的 setJointValue。
 * 页面入口：index.html 末尾的 <script type="module" src="./js/analysis/analysis-panel.js">。
 */
import * as THREE from 'three';
import { setJointValue } from '../urdf-model.js';
import { TrajectoryCharts } from './TrajectoryCharts.js';
import { WorkspaceCloud } from './WorkspaceCloud.js';

const $ = (id) => document.getElementById(id);
const charts = new TrajectoryCharts({ windowSeconds: 12, unitLabel: '单位' });

let api = null;
let points = null;
let recording = false;
let startTime = 0;
let lastStamp = 0;
const current = new THREE.Vector3();
const last = new THREE.Vector3();
let hasLast = false;

// 组员B_工位验证数据.json 中的工件原点 O（世界坐标）
const WORKPIECE_O = new THREE.Vector3(6.86912072988322, 4.412209498388784, -0.12956336275712083);

function format(value, digits = 3) {
  return Number(value).toFixed(digits);
}

function setStatus(id, text) {
  const node = $(id);
  if (node) node.textContent = text;
}

function updateStats() {
  const stats = charts.getStats();
  $('tc-count').textContent = `${stats.count}`;
  $('tc-duration').textContent = `${format(stats.duration, 2)} s`;
  $('tc-vmax').textContent = format(stats.maxSpeed);
  $('tc-vavg').textContent = format(stats.avgSpeed);
  $('tc-path').textContent = format(stats.pathLength);
}

function tick() {
  requestAnimationFrame(tick);
  if (!recording || !api) return;
  const stamp = performance.now() / 1000;
  api.tcp.getWorldPosition(current);
  const speed = hasLast && stamp > lastStamp ? current.distanceTo(last) / (stamp - lastStamp) : 0;
  last.copy(current);
  lastStamp = stamp;
  hasLast = true;
  charts.recordSample(stamp - startTime, { x: current.x, y: current.y, z: current.z }, speed);
  charts.update();
  if (statsTimer > 6) {
    statsTimer = 0;
    updateStats();
  }
  statsTimer += 1;

  // 程序跑完自动停止记录
  const runner = api.program?.runner;
  if (runner && !runner.state.running && recording && autoStopWhenProgramEnds) {
    setTimeout(() => { if (recording && !runner.state.running) stopRecording('程序结束，已自动停止记录'); }, 400);
    autoStopWhenProgramEnds = false;
  }
}

let statsTimer = 0;
let autoStopWhenProgramEnds = false;

function startRecording() {
  if (!api) return;
  charts.clear();
  charts.follow = true;
  recording = true;
  startTime = performance.now() / 1000;
  lastStamp = startTime;
  hasLast = false;
  autoStopWhenProgramEnds = Boolean(api.program?.runner?.state.running);
  setStatus('tc-status', '记录中… 执行「示教编程」里的程序，曲线会实时增长。');
  updateStats();
}

function stopRecording(message) {
  recording = false;
  charts.follow = false;
  setStatus('tc-status', message ?? `已停止：共 ${charts.count} 个采样点。`);
  updateStats();
  charts.draw();
}

async function generateCloud() {
  if (!api) return;
  const count = Math.max(1000, Number($('wc-input').value) || 20000);
  const limits = api.joints.map((joint) => ({ min: joint.lower, max: joint.upper }));
  const backup = api.joints.map((joint) => joint.angle);

  const fk = (angles) => {
    for (let index = 0; index < api.joints.length; index += 1) {
      setJointValue(api.joints[index], angles[index]);
    }
    api.tcp.updateWorldMatrix(true, false);
    return api.tcp.getWorldPosition(new THREE.Vector3());
  };

  setStatus('wc-status', `采样中（${count} 点）…`);
  await new Promise((resolve) => setTimeout(resolve, 50));

  const started = performance.now();
  const cloud = WorkspaceCloud.generatePointCloud(fk, limits, count, { seed: 20261026 });
  const cost = performance.now() - started;

  api.joints.forEach((joint, index) => setJointValue(joint, backup[index]));
  api.robot.updateMatrixWorld(true);

  if (points) {
    points.parent?.remove(points);
    points.geometry.dispose();
    points.material.dispose();
  }
  points = cloud;
  points.visible = $('wc-visible').checked !== false;
  (api.robot.parent ?? api.robot).add(points);

  const geometry = WorkspaceCloud.analyzeWorkspaceGeometry(points, { unitLabel: '单位' });
  const cavities = WorkspaceCloud.detectCavities(points);

  $('wc-points').textContent = `${points.geometry.attributes.position.count} 点`;
  $('wc-radius').textContent = `${format(geometry.reachRadiusMin)} ~ ${format(geometry.reachRadiusMax)}`;
  $('wc-height').textContent = `${format(geometry.zHeightRange[0])} ~ ${format(geometry.zHeightRange[1])}`;
  $('wc-void').textContent = `${(cavities.voidRatio * 100).toFixed(1)} %`;
  $('wc-hole').textContent = cavities.largest
    ? `${cavities.largest.cells} 格`
    : '未检出';

  // 工件原点：优先用场景里真实的 mark-O 标记点（工位场景），找不到再退回张云乔的验证数据
  const scene = api.robot.parent ?? api.robot;
  const marker = scene.getObjectByName('mark-O');
  const workpieceO = marker ? marker.getWorldPosition(new THREE.Vector3()) : WORKPIECE_O;

  const array = points.geometry.attributes.position.array;
  let nearest = Infinity;
  for (let index = 0; index < array.length; index += 3) {
    const distance = Math.hypot(array[index] - workpieceO.x, array[index + 1] - workpieceO.y, array[index + 2] - workpieceO.z);
    if (distance < nearest) nearest = distance;
  }
  $('wc-workpiece').textContent = `${format(nearest)}（${nearest < 0.15 ? '可达区内' : nearest < 0.4 ? '边缘' : '区域外'}）`;

  setStatus('wc-status', `完成：${points.geometry.attributes.position.count} 点，用时 ${cost.toFixed(0)} ms。\n${geometry.evaluationText}`);
}

function bind() {
  window.addEventListener('robot-scan-start', startRecording);
  charts.renderCharts($('tc-chart'));
  updateStats();

  $('tc-start')?.addEventListener('click', startRecording);
  $('tc-stop')?.addEventListener('click', () => stopRecording());
  $('tc-clear')?.addEventListener('click', () => { charts.clear(); charts.draw(); updateStats(); setStatus('tc-status', '已清空。'); });
  $('tc-csv')?.addEventListener('click', () => {
    if (!charts.count) { setStatus('tc-status', '还没有数据，先开始记录。'); return; }
    const rows = charts.downloadCSV('轨迹数据_王静.csv');
    setStatus('tc-status', `已导出 ${rows} 行 CSV。`);
  });

  $('wc-generate')?.addEventListener('click', generateCloud);
  $('wc-remove')?.addEventListener('click', () => {
    if (points) { points.parent?.remove(points); }
    setStatus('wc-status', '已从场景移除点云。');
  });
  $('wc-visible')?.addEventListener('change', (event) => { if (points) points.visible = event.target.checked; });

  for (const id of ['tab-analysis', 'tab-cloud']) {
    $(id)?.addEventListener('click', () => requestAnimationFrame(() => charts.draw()));
  }

  const hash = (location.hash || '').replace('#', '');
  if (hash === 'analysis') $('tab-analysis')?.click();
  if (hash === 'cloud') $('tab-cloud')?.click();

  // 本地调试用：控制台可查看，或自动导出 CSV
  window.__analysisLab = { charts, get points() { return points; } };
}

function waitForRobot() {
  const timer = setInterval(() => {
    if (window.robotSimulation?.tcp) {
      clearInterval(timer);
      api = window.robotSimulation;
      bind();
      tick();
    }
  }, 300);
}

waitForRobot();




