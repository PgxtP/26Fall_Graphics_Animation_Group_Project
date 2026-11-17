import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';
import { createDSLRunner, parseDSLScript, DSL_STATUS } from './dsl-interpreter.js';
import { poseFromMatrix } from './movl-planner.js';
import { createCartesianScan } from './cartesian-scan.js';

export function bindProgram(api, motion) {
  const poses = [];
  // MOVL 需要把「已记录位姿」换算成 TCP 世界位姿：临时写关节角读矩阵，立刻恢复，
  // 与 cartesian-scan.js:67-70 一样保证规划期画面不动（无副作用）。
  const poseOfAngles = (angles) => {
    const backup = api.joints.map((joint) => joint.angle);
    try {
      angles.forEach((value, index) => setJointValue(api.joints[index], value));
      api.robot.updateMatrixWorld(true);
      api.tcp.updateWorldMatrix(true, false);
      return poseFromMatrix(api.tcp.matrixWorld.elements);
    } finally {
      backup.forEach((value, index) => setJointValue(api.joints[index], value));
      api.robot.updateMatrixWorld(true);
    }
  };
  // 执行器换成 DSL 版本：state.running / instructions / index / message / update() 与旧版一致，
  // 纯 MOVJ 文本的行为保持不变（对外仍暴露 api.program.runner）。
  const runner = createDSLRunner(api, motion, { poseOfAngles, movlSpeed: 1 });
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
  const action = (id, fn) => {
    const element = document.getElementById(id);
    if (!element) return;
    element.addEventListener('click', () => {
      try { fn(); } catch (error) { status.textContent = error.message; }
    });
  };
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
    runner.start(code.value, poses, { poseOfAngles });
    if (activeScan) window.dispatchEvent(new Event('robot-scan-start'));
    status.textContent = runner.state.message;
  });
  action('stop-program', () => { runner.stop(); status.textContent = runner.state.message; });

  // --- 第 4 章新增：工业 DSL 面板（动态注入，不改 index.html / css/style.css） ---
  const buildDSLPanel = () => {
    const host = document.getElementById('task-program');
    if (!host || document.getElementById('dsl-panel')) return;
    const panel = document.createElement('section');
    panel.id = 'dsl-panel';
    panel.className = 'step';
    panel.style.marginTop = '16px';
    panel.innerHTML = [
      '<h3>4. 工业 DSL 程序（MOVL 直线插补 / 夹爪 / 延时 / 循环）</h3>',
      '<p class="hint">每行一条指令：<code>MOVJ P1</code> 关节插值；<code>MOVL P2</code> TCP 走直线；',
      '<code>GRIP 1</code> 夹紧、<code>GRIP 0</code> 松开；<code>WAIT 1.5</code> 等待；',
      '<code>SET SPEED 40</code> 改速度倍率；<code>LOOP 3 … END</code> 循环。</p>',
      '<textarea id="dsl-code" rows="8" spellcheck="false" placeholder="# 示例&#10;SET SPEED 40&#10;MOVJ P1&#10;MOVL P2&#10;GRIP 1&#10;WAIT 0.5&#10;LOOP 2&#10;MOVL P3&#10;GRIP 0&#10;END"></textarea>',
      '<div class="button-row">',
      '<button id="dsl-example" class="secondary">插入 MOVL 示例</button>',
      '<button id="dsl-check" class="secondary">检查语法</button>',
      '<button id="dsl-run">执行 DSL 程序</button>',
      '<button id="dsl-pause" class="secondary">暂停</button>',
      '<button id="dsl-resume" class="secondary">继续</button>',
      '</div>',
      '<p id="dsl-status" class="result" role="status">状态机：IDLE · 尚未执行</p>',
      '<details><summary>查看展开后的指令表（LOOP 已展开）</summary><ol id="dsl-instructions"></ol></details>',
    ].join('');
    host.appendChild(panel);
  };
  buildDSLPanel();

  const dslCode = document.getElementById('dsl-code');
  const dslStatusEl = document.getElementById('dsl-status');
  const dslInstructionList = document.getElementById('dsl-instructions');
  const dslExample = [
    'SET SPEED 40',
    'MOVJ P1',
    'MOVL P2',
    'GRIP 1',
    'WAIT 0.5',
    'LOOP 2',
    'MOVL P3',
    'GRIP 0',
    'END',
  ].join('\n');

  const describeInstruction = (instruction) => {
    switch (instruction.type) {
      case 'MOVJ': return `MOVJ ${instruction.name}`;
      case 'MOVL': return `MOVL ${instruction.name}`;
      case 'GRIP': return `GRIP ${instruction.value}`;
      case 'WAIT': return `WAIT ${instruction.seconds} 秒`;
      case 'SPEED': return `SET SPEED ${instruction.percent}%`;
      default: return instruction.type;
    }
  };

  const renderDSLInstructions = (instructions) => {
    if (!dslInstructionList) return;
    dslInstructionList.replaceChildren();
    instructions.forEach((instruction) => {
      const item = document.createElement('li');
      item.textContent = `第 ${instruction.line} 行 · ${describeInstruction(instruction)}`;
      dslInstructionList.appendChild(item);
    });
  };

  let holdDSLStatus = false;
  const showDSLParse = () => {
    if (!dslCode || !dslStatusEl) return;
    holdDSLStatus = true;
    const parsed = parseDSLScript(dslCode.value, poses);
    if (parsed.ok) {
      dslStatusEl.textContent = `语法检查通过：展开后 ${parsed.instructions.length} 条指令，引用位姿 ${parsed.poseNames.join('、') || '无'}`;
      renderDSLInstructions(parsed.instructions);
      return;
    }
    dslStatusEl.textContent = `语法检查未通过：${parsed.errors.map((error) => error.message).join('；')}`;
    renderDSLInstructions([]);
  };

  action('dsl-example', () => {
    if (!dslCode) return;
    dslCode.value = dslExample;
    showDSLParse();
    status.textContent = '已插入 DSL 示例；请确认 P1～P3 已记录，再点「执行 DSL 程序」';
  });
  action('dsl-check', () => showDSLParse());
  action('dsl-run', () => {
    if (!dslCode) return;
    try {
      holdDSLStatus = false;
      runner.start(dslCode.value, poses, { poseOfAngles });
      renderDSLInstructions(runner.state.instructions);
      status.textContent = runner.state.message;
      if (dslStatusEl) {
        if (runner.state.running) {
          dslStatusEl.textContent = '状态机：RUNNING · ' + runner.state.message;
        } else {
          holdDSLStatus = true;
          dslStatusEl.textContent = `状态机：${runner.state.status} · ${runner.state.message}`;
        }
      }
      previousDSLStatus = '';
    } catch (error) {
      holdDSLStatus = true;
      if (dslStatusEl) dslStatusEl.textContent = '无法执行：' + error.message;
      throw error;
    }
  });
  action('dsl-pause', () => {
    if (!runner.pause()) { status.textContent = '当前没有正在运行的 DSL 程序，无需暂停'; return; }
    status.textContent = runner.state.message;
  });
  action('dsl-resume', () => {
    if (!runner.resume()) { status.textContent = '程序不在暂停状态，无需继续'; return; }
    status.textContent = runner.state.message;
  });

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

  api.program = {
    runner, poses,
    get code() { return code.value; },
    get dslCode() { return dslCode?.value ?? ''; },
    parseDSL: (text) => parseDSLScript(text ?? dslCode?.value ?? '', poses),
  };
  let previousDSLStatus = '';
  return { update() {
    runner.update();
    if (runner.state.message !== previousMessage) { previousMessage = runner.state.message; status.textContent = previousMessage; }
    const running = runner.state.running;
    const paused = runner.state.status === DSL_STATUS.PAUSED;
    for (const id of ['record-pose', 'undo-pose', 'clear-poses', 'generate-movj', 'scan-program']) document.getElementById(id).disabled = running;
    code.readOnly = running;
    document.getElementById('run-program').disabled = running || !motion.allowed();
    if (dslCode) dslCode.readOnly = running;
    const dslRun = document.getElementById('dsl-run');
    if (dslRun) dslRun.disabled = running;
    const dslPause = document.getElementById('dsl-pause');
    if (dslPause) dslPause.disabled = runner.state.status !== DSL_STATUS.RUNNING;
    const dslResume = document.getElementById('dsl-resume');
    if (dslResume) dslResume.disabled = !paused;
    if (dslStatusEl) {
      if (running) holdDSLStatus = false;
      if (!holdDSLStatus) {
        const total = runner.state.instructions.length;
        const line = runner.state.instructions[runner.state.index]?.line;
        const text = `状态机：${runner.state.status}`
          + (total ? ` · 第 ${Math.min(runner.state.index + 1, total)}/${total} 条` : '')
          + (line ? ` · 源文件第 ${line} 行` : '')
          + (runner.state.lastMOVL?.ok ? ` · MOVL ${runner.state.lastMOVL.sampleCount} 个插补点` : '')
          + (!running && runner.state.status === DSL_STATUS.IDLE && runner.state.message ? ` · ${runner.state.message}` : '');
        if (text !== previousDSLStatus) { previousDSLStatus = text; dslStatusEl.textContent = text; }
      }
    }
    [...table.rows].forEach((row) => {
      const name = runner.state.instructions[runner.state.index]?.name;
      row.classList.toggle('active-pose', running && row.cells[0].textContent === name);
    });
  } };
}



