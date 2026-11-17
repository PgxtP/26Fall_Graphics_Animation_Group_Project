import * as THREE from 'three';
import { createProgramRunner } from './movj-program.js';
import { createCartesianScan } from './cartesian-scan.js';

export function bindProgram(api, motion) {
  const poses = [], runner = createProgramRunner(api, motion);
  let activeScan = null;
  let scanGuide = null;
  const showScanGuide = (scan) => {
    if (scanGuide) {
      scanGuide.parent?.remove(scanGuide);
      scanGuide.traverse((node) => { node.geometry?.dispose(); node.material?.dispose(); });
      scanGuide = null;
    }
    if (!scan || !Array.isArray(scan.center) || !Number.isFinite(scan.radius)) return;
    const center = new THREE.Vector3(...scan.center);
    const points = Array.from({ length: 96 }, (_, index) => {
      const angle = index / 96 * Math.PI * 2;
      return center.clone().add(new THREE.Vector3(scan.radius * Math.cos(angle), 0, scan.radius * Math.sin(angle)));
    });
    scanGuide = new THREE.Group();
    scanGuide.name = 'scan-guide';
    scanGuide.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: 0xffd166, depthTest: false })));
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xff7043, depthTest: false }));
    marker.position.copy(center);
    scanGuide.add(marker);
    api.robot.parent?.add(scanGuide);
  };
  const STORAGE_KEY = 'robot-arm-program-v1';
  const PROGRAM_TYPE = 'robot-arm-program';
  const MODEL_ID = 'hanz_robot_5dof';
  const code = document.getElementById('movj-code'), status = document.getElementById('program-status');
  const table = document.getElementById('pose-list');
  const requireStopped = () => { if (runner.state.running || motion.busy()) throw new Error('请先停止运动，再记录或修改位姿'); };
  const render = () => {
    table.replaceChildren();
    for (const pose of poses) {
      const row = document.createElement('tr');
      for (const value of [pose.name, ...pose.angles.map((v) => THREE.MathUtils.radToDeg(v).toFixed(1))]) {
        const td = document.createElement('td'); td.textContent = value; row.appendChild(td);
      }
      table.appendChild(row);
    }
    document.getElementById('pose-count').textContent = `已记录 ${poses.length} 个位姿（至少需要4个）`;
  };
  const action = (id, fn) => document.getElementById(id).addEventListener('click', () => {
    try { fn(); } catch (error) { status.textContent = error.message; }
  });
  action('record-pose', () => {
    requireStopped();
    api.robot.updateMatrixWorld(true);
    activeScan = null;
    showScanGuide(null);
    poses.push({ name: `P${poses.length + 1}`, angles: api.joints.map((j) => j.angle), tcpWorld: api.tcp.matrixWorld.toArray() });
    render(); saveRecord(); status.textContent = `已记录 ${poses.at(-1).name}；继续点动到下一个姿态后记录`;
  });
  action('undo-pose', () => { requireStopped(); activeScan = null; showScanGuide(null); poses.pop(); render(); saveRecord(); status.textContent = '已撤销最后一个位姿，请重新生成程序'; });
  action('clear-poses', () => { requireStopped(); activeScan = null; showScanGuide(null); poses.length = 0; code.value = ''; render(); saveRecord(); status.textContent = '位姿与程序已清空'; });
  action('generate-movj', () => { requireStopped(); code.value = poses.map((pose) => `MOVJ ${pose.name}`).join('\n'); status.textContent = '已按位姿记录顺序生成 MOVJ 程序'; saveRecord();});
  action('scan-program', () => {
    requireStopped();
    const scan = createCartesianScan(api);
    const data = {
      version: 1, type: PROGRAM_TYPE, model: MODEL_ID, unit: 'deg',
      scan: {
        kind: 'tcp-horizontal-circle',
        center: scan.center,
        radius: scan.radius,
        segments: scan.segments,
        maxWaypointError: scan.maxWaypointError,
        tcpOffsetMatrix: api.calibration.tcpOffsetMatrix.toArray(),
      },
      poses: scan.poses.map((pose) => ({
        name: pose.name,
        anglesDeg: pose.angles.map(THREE.MathUtils.radToDeg),
        tcpWorld: pose.tcpWorld,
        targetWorld: pose.targetWorld,
      })),
      code: scan.poses.map((pose) => `MOVJ ${pose.name}`).join('\n'),
    };
    applyRecord(data, '预设');
    status.textContent = `已载入 ${scan.poses.length} 点 TCP 环绕扫描；半径 ${scan.radius}，J5 同步旋转 360°。请先使能再执行。`;
  });
  action('run-program', () => {
    if (activeScan && api.calibration.tcpOffsetMatrix.toArray().some((value, index) =>
      Math.abs(value - activeScan.tcpOffsetMatrix[index]) > 1e-8)) {
      throw new Error('TCP 标定已变化，请重新载入 360° 扫描预设');
    }
    runner.start(code.value, poses);
    if (activeScan) window.dispatchEvent(new Event('robot-scan-start'));
    status.textContent = runner.state.message;
  });
  action('stop-program', () => { runner.stop(); status.textContent = runner.state.message; });
  let previousMessage = '';
  render();

  // --- 数据管理与本地存储 ---
  const record = (source = 'manual') => ({
    version: 1, type: PROGRAM_TYPE, model: MODEL_ID, unit: 'deg',
    source, savedAt: new Date().toISOString(),
    ...(activeScan ? { scan: activeScan } : {}),
    poses: poses.map(p => ({
      name: p.name,
      anglesDeg: p.angles.map(THREE.MathUtils.radToDeg),
      ...(Array.isArray(p.tcpWorld) ? { tcpWorld: [...p.tcpWorld] } : {}),
      ...(Array.isArray(p.targetWorld) ? { targetWorld: [...p.targetWorld] } : {})
    })),
    code: code.value,
  });

  const saveRecord = () => {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(record('localStorage'))); } catch {}
  };

  const applyRecord = (data, label) => {
    requireStopped();
    if (!data || data.version !== 1 || data.type !== PROGRAM_TYPE) throw new Error('格式错误');
    if (!Array.isArray(data.poses)) throw new Error('位姿数据无效');
    activeScan = data.scan?.kind === 'tcp-horizontal-circle' &&
      Array.isArray(data.scan.tcpOffsetMatrix) && data.scan.tcpOffsetMatrix.length === 16 ? data.scan : null;
    poses.length = 0;
    for (const p of data.poses) {
      if (!Array.isArray(p.anglesDeg) || p.anglesDeg.length !== 5) throw new Error('角度数据无效');
      poses.push({ name: p.name, angles: p.anglesDeg.map(THREE.MathUtils.degToRad), tcpWorld: p.tcpWorld, targetWorld: p.targetWorld });
    }
    code.value = data.code || poses.map(p => `MOVJ ${p.name}`).join('\n');
    showScanGuide(activeScan);
    render(); saveRecord();
    status.textContent = `${label}：已恢复 ${poses.length} 个位姿与代码。`;
  };

  action('export-program', () => {
    requireStopped();
    if (poses.length === 0) throw new Error('没有可导出的位姿');
    const url = URL.createObjectURL(new Blob([JSON.stringify(record('export'), null, 2)], { type: 'application/json' }));
    const a = document.createElement('a');
    a.href = url; a.download = `robot-program-${Date.now()}.json`;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    status.textContent = '位姿程序 JSON 已导出下载。';
  });

  const importBtn = document.getElementById('import-program-button');
  const importInput = document.getElementById('import-program');
  if (importBtn && importInput) {
    importBtn.addEventListener('click', () => importInput.click());
    importInput.addEventListener('change', async (e) => {
      try {
        const file = e.target.files[0];
        if (!file) return;
        const text = await file.text();
        applyRecord(JSON.parse(text), '导入');
      } catch (err) {
        status.textContent = '导入失败：' + err.message;
      } finally { e.target.value = ''; }
    });
  }

  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) applyRecord(JSON.parse(saved), '本地恢复');
  } catch {}

  api.program = { runner, poses, get code() { return code.value; } };
  return { update() {
    runner.update();
    if (runner.state.message !== previousMessage) { previousMessage = runner.state.message; status.textContent = previousMessage; }
    const running = runner.state.running;
    for (const id of ['record-pose', 'undo-pose', 'clear-poses', 'generate-movj', 'scan-program']) document.getElementById(id).disabled = running;
    code.readOnly = running;
    document.getElementById('run-program').disabled = running || !motion.allowed();
    [...table.rows].forEach((row) => {
      const name = runner.state.instructions[runner.state.index]?.name;
      row.classList.toggle('active-pose', running && row.cells[0].textContent === name);
    });
  } };
}



