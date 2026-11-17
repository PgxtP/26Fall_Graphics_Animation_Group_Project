import * as THREE from 'three';
import { createMotion } from './pendant-motion.js';
import { solveIK } from './ik-solver.js';

export const JOG_MODES = Object.freeze(['Joint', 'Base', 'Tool', 'User']);

const AXES = Object.freeze({
  X: new THREE.Vector3(1, 0, 0),
  Y: new THREE.Vector3(0, 1, 0),
  Z: new THREE.Vector3(0, 0, 1),
});

function normalizeJogMode(mode) {
  const value = String(mode ?? '').trim().toLowerCase();
  const matched = JOG_MODES.find((item) => item.toLowerCase() === value);
  if (!matched) throw new Error(`不支持的点动模式：${mode}`);
  return matched;
}

function cartesianDirection(api, mode, axis) {
  const direction = AXES[axis].clone();
  api.robot?.updateMatrixWorld?.(true);

  if (mode === 'Base') {
    const base = api.robot?.getObjectByName?.('link_base');
    if (!base) throw new Error('未找到 link_base，无法按基座坐标系点动');
    return direction.transformDirection(base.matrixWorld);
  }
  if (mode === 'Tool') {
    api.tcp?.updateWorldMatrix?.(true, false);
    if (!api.tcp?.matrixWorld) throw new Error('未找到 TCP，无法按工具坐标系点动');
    return direction.transformDirection(api.tcp.matrixWorld);
  }
  if (mode === 'User') {
    const worldToUser = api.calibration?.userMatrix;
    if (!worldToUser) throw new Error('尚未建立用户坐标系，无法使用 User 点动');
    return direction.transformDirection(worldToUser.clone().invert());
  }
  throw new Error('Joint 模式请使用关节点动按钮');
}

/** 返回 TCP 在当前点动坐标系下的位置与 RPY（弧度，ZYX 固定轴约定）。 */
export function getTCPPoseInJogFrame(api, requestedMode) {
  const mode = normalizeJogMode(requestedMode);
  api.robot?.updateMatrixWorld?.(true);
  api.tcp?.updateWorldMatrix?.(true, false);
  if (!api.tcp?.matrixWorld) throw new Error('未找到 TCP，无法更新位姿 HUD');

  const tcpWorld = api.tcp.matrixWorld.clone();
  let relativeMatrix;
  let frameLabel;
  if (mode === 'User') {
    const worldToUser = api.calibration?.userMatrix;
    if (!worldToUser) throw new Error('尚未建立用户坐标系');
    relativeMatrix = worldToUser.clone().multiply(tcpWorld);
    frameLabel = 'User';
  } else if (mode === 'Tool') {
    relativeMatrix = new THREE.Matrix4();
    frameLabel = 'Tool（当前 TCP）';
  } else {
    const base = api.robot?.getObjectByName?.('link_base');
    if (!base) throw new Error('未找到 link_base，无法更新位姿 HUD');
    relativeMatrix = base.matrixWorld.clone().invert().multiply(tcpWorld);
    frameLabel = mode === 'Joint' ? 'Base（Joint 模式 HUD）' : 'Base';
  }

  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  relativeMatrix.decompose(position, quaternion, new THREE.Vector3());
  const rpy = new THREE.Euler().setFromQuaternion(quaternion.normalize(), 'ZYX');
  return { mode, frameLabel, position, quaternion, rpy };
}

function createCartesianJogPanel(api, motion, jogController, motionMessage) {
  const panel = document.createElement('section');
  panel.className = 'cartesian-jog';
  panel.setAttribute('aria-labelledby', 'cartesian-jog-title');
  panel.innerHTML = `
    <div class="cartesian-jog-heading">
      <h2 id="cartesian-jog-title">多坐标系点动</h2>
      <output id="cartesian-frame-label">Base（Joint 模式 HUD）</output>
    </div>
    <label for="coordinate-jog-mode">坐标形式
      <select id="coordinate-jog-mode" aria-label="坐标点动模式">
        ${JOG_MODES.map((mode) => `<option value="${mode}">${mode}</option>`).join('')}
      </select>
    </label>
    <div class="cartesian-pose-grid" aria-label="当前坐标系下的 TCP 位姿">
      ${[
        ['X', 'x', 'TCP X'], ['Y', 'y', 'TCP Y'], ['Z', 'z', 'TCP Z'],
        ['R', 'r', 'TCP Roll'], ['P', 'p', 'TCP Pitch'], ['Y', 'yaw', 'TCP Yaw'],
      ].map(([label, key, ariaLabel]) => `<label>${label}<input id="tcp-hud-${key}" type="text" value="—" readonly aria-label="${ariaLabel}"></label>`).join('')}
    </div>
    <p class="cartesian-unit">X/Y/Z：模型单位　R/P/Y：度</p>
    <label for="cartesian-step">位置点动步长（模型单位）
      <input id="cartesian-step" type="number" min="0.001" step="0.01" value="0.05">
    </label>
    <div class="cartesian-jog-buttons" aria-label="笛卡尔位置点动">
      ${['X', 'Y', 'Z'].flatMap((axis) => [-1, 1].map((direction) => `<button type="button" class="compact ${direction < 0 ? 'secondary' : ''}" data-axis="${axis}" data-direction="${direction}" aria-label="${axis} ${direction < 0 ? '负向' : '正向'}笛卡尔点动">${direction < 0 ? '−' : '+'}${axis}</button>`)).join('')}
    </div>
    <p id="cartesian-status" class="cartesian-status" role="status" aria-live="polite">Joint 模式使用下方关节按钮；HUD 按 Base 坐标显示。</p>`;

  document.getElementById('joint-controls').before(panel);
  const modeSelect = panel.querySelector('#coordinate-jog-mode');
  const stepInput = panel.querySelector('#cartesian-step');
  const status = panel.querySelector('#cartesian-status');
  const frameLabel = panel.querySelector('#cartesian-frame-label');
  const poseFields = ['x', 'y', 'z', 'r', 'p', 'yaw'].map((name) => panel.querySelector(`#tcp-hud-${name}`));
  const jogButtons = [...panel.querySelectorAll('[data-axis]')];

  const setStatus = (text, alarm = false) => {
    status.textContent = text;
    status.classList.toggle('alarm', alarm);
    status.setAttribute('role', alarm ? 'alert' : 'status');
    status.setAttribute('aria-live', alarm ? 'assertive' : 'polite');
  };

  const updateHud = () => {
    try {
      const wasReady = panel.dataset.hudReady;
      const pose = getTCPPoseInJogFrame(api, jogController.getJogMode());
      const values = [
        pose.position.x.toFixed(3),
        pose.position.y.toFixed(3),
        pose.position.z.toFixed(3),
        THREE.MathUtils.radToDeg(pose.rpy.x).toFixed(2),
        THREE.MathUtils.radToDeg(pose.rpy.y).toFixed(2),
        THREE.MathUtils.radToDeg(pose.rpy.z).toFixed(2),
      ];
      poseFields.forEach((field, index) => { field.value = values[index]; });
      frameLabel.textContent = pose.frameLabel;
      panel.dataset.hudReady = 'true';
      if (wasReady === 'false' && !status.classList.contains('alarm')) {
        setStatus(`${jogController.getJogMode()} 坐标点动已就绪。`);
      }
    } catch (error) {
      poseFields.forEach((field) => { field.value = '—'; });
      frameLabel.textContent = jogController.getJogMode();
      panel.dataset.hudReady = 'false';
      if (!status.classList.contains('alarm')) setStatus(error.message);
    }
  };

  const updateAvailability = () => {
    const mode = jogController.getJogMode();
    const manualAllowed = motion.allowed() && !motion.state.programActive;
    const cartesianReady = mode !== 'Joint' && panel.dataset.hudReady === 'true';
    modeSelect.disabled = motion.state.programActive;
    stepInput.disabled = !manualAllowed || mode === 'Joint';
    jogButtons.forEach((button) => { button.disabled = !manualAllowed || !cartesianReady; });
  };

  modeSelect.addEventListener('change', () => {
    try {
      const mode = jogController.switchJogMode(modeSelect.value);
      setStatus(mode === 'Joint'
        ? 'Joint 模式使用下方关节按钮；HUD 按 Base 坐标显示。'
        : `${mode} 坐标点动已选中。`);
      updateHud();
      updateAvailability();
    } catch (error) {
      modeSelect.value = jogController.getJogMode();
      setStatus(error.message, true);
    }
  });

  for (const button of jogButtons) {
    button.addEventListener('click', () => {
      try {
        const step = Number(stepInput.value);
        if (!Number.isFinite(step) || step <= 0) throw new Error('位置点动步长必须为正有限数');
        const signedStep = Number(button.dataset.direction) * step;
        const result = jogController.jogStepCartesian(button.dataset.axis, signedStep);
        setStatus(`已下发 ${result.mode} ${signedStep < 0 ? '−' : '+'}${result.axis} 点动，IK：${result.method ?? 'auto'}。`);
        motionMessage.textContent = '笛卡尔点动正在执行。可随时按“停止运动”或“急停”。';
      } catch (error) {
        setStatus(`报警：${error.message}`, true);
        motionMessage.textContent = error.message;
      }
    });
  }

  return {
    setStatus,
    update() {
      updateHud();
      updateAvailability();
    },
  };
}

/**
 * 创建四种点动模式的核心控制器。此处不创建界面，供后续笛卡尔面板调用。
 * step 使用当前模型的世界坐标单位；正负号决定沿所选坐标轴的方向。
 */
export function createJogController(api, options = {}) {
  if (!api?.robot || !api?.tcp || !Array.isArray(api.joints)) {
    throw new Error('点动控制器需要 robot、tcp 和 joints 接口');
  }

  let mode = 'Joint';
  const canMove = options.canMove ?? (() => true);
  const applyAngles = options.applyAngles ?? (() => {});
  const stopMotion = options.stopMotion ?? (() => {});
  const runIK = options.solveIK ?? ((targetPosition, targetRotation, currentJoints, ikOptions) => {
    if (typeof api.solveIK === 'function') {
      return api.solveIK(targetPosition, targetRotation, currentJoints, ikOptions);
    }
    return solveIK(api, targetPosition, targetRotation, currentJoints, ikOptions);
  });

  const switchJogMode = (nextMode) => {
    const normalized = normalizeJogMode(nextMode);
    if (normalized !== mode) stopMotion();
    mode = normalized;
    return mode;
  };

  const jogStepCartesian = (axis, step) => {
    const normalizedAxis = String(axis ?? '').trim().toUpperCase();
    if (!AXES[normalizedAxis]) throw new Error(`不支持的笛卡尔轴：${axis}`);
    if (mode === 'Joint') throw new Error('Joint 模式请使用关节点动按钮');
    if (!Number.isFinite(step) || step === 0) throw new Error('笛卡尔点动步长必须是非零有限数值');
    if (!canMove()) throw new Error('请先解除急停并打开关节使能');

    api.robot.updateMatrixWorld(true);
    api.tcp.updateWorldMatrix(true, false);
    const targetPosition = api.tcp.getWorldPosition(new THREE.Vector3());
    const targetRotation = api.tcp.getWorldQuaternion(new THREE.Quaternion());
    targetPosition.addScaledVector(cartesianDirection(api, mode, normalizedAxis), step);

    const currentJoints = api.joints.map((joint) => joint.angle);
    // 本面板只提供 ±X/±Y/±Z 位置微调。五轴机械臂无法独立满足完整六维位姿约束，
    // 因此按组长接口约定传 null，执行仅位置 3D 逆解；R/P/Y 仅由 HUD 实时显示。
    const result = runIK(targetPosition, null, currentJoints, { method: 'auto' });
    if (!result || !Array.isArray(result.angles)) throw new Error('IK 返回结果格式无效，已拦截运动');
    if (!result.converged) throw new Error('IK 未收敛，目标不可达，已拦截运动');
    if (result.singular) throw new Error('IK 检测到奇异位形，已拦截运动');
    if (result.angles.length !== api.joints.length || result.angles.some((angle) => !Number.isFinite(angle))) {
      throw new Error('IK 返回的关节角数量或数值无效，已拦截运动');
    }
    if (result.angles.some((angle, index) => angle < api.joints[index].lower - 1e-9 || angle > api.joints[index].upper + 1e-9)) {
      throw new Error('IK 结果超出关节限位，已拦截运动');
    }

    const angles = [...result.angles];
    applyAngles(angles);
    return { ...result, angles, mode, axis: normalizedAxis, step, targetPosition, targetRotation };
  };

  return { switchJogMode, jogStepCartesian, getJogMode: () => mode };
}

export function bindJointControls(api, urdfModel) {
  const motion = createMotion(api, urdfModel);
  const jogController = createJogController(api, {
    canMove: () => motion.allowed() && !motion.state.programActive,
    applyAngles: (angles) => motion.moveJ(angles),
    stopMotion: motion.stop,
  });
  const message = document.getElementById('motion-status');
  const cartesianPanel = createCartesianJogPanel(api, motion, jogController, message);
  const controls = [];
  const act = (fn) => { try { fn(); message.textContent = '正在执行。可随时按“停止运动”或“急停”。'; } catch (error) { message.textContent = error.message; } };
  for (const joint of api.joints) {
    const group = document.createElement('div'); group.className = 'joint-control';
    const label = document.createElement('label'); label.htmlFor = `angle-${joint.name}`;
    label.textContent = `${joint.name}　${({ J1: '肩部', J2: '肘部俯仰', J3: '肘部横摆', J4: '腕部', J5: '法兰' })[joint.name]} (${joint.lowerDeg}°～${joint.upperDeg}°)`;
    const output = document.createElement('output'); output.textContent = '0.0°';
    const slider = document.createElement('input'); slider.type = 'range'; slider.id = label.htmlFor;
    const number = document.createElement('input'); number.type = 'number'; number.id = `number-${joint.name}`; number.setAttribute('aria-label', `${joint.name}目标角度`);
    for (const input of [slider, number]) {
      input.min = joint.lowerDeg; input.max = joint.upperDeg; input.step = '0.1'; input.value = '0';
    }
    slider.addEventListener('input', () => act(() => {
      number.value = slider.value;
      motion.target(joint.name, THREE.MathUtils.degToRad(slider.valueAsNumber));
    }));
    slider.addEventListener('change', () => {
      slider.value = THREE.MathUtils.radToDeg(joint.angle);
      number.value = slider.value;
    });
    const go = document.createElement('button'); go.textContent = '到位'; go.className = 'compact';
    go.addEventListener('click', () => act(() => motion.target(joint.name, THREE.MathUtils.degToRad(number.valueAsNumber))));
    const buttons = document.createElement('div'); buttons.className = 'button-row';
    for (const direction of [-1, 1]) {
      const button = document.createElement('button'); button.textContent = direction < 0 ? '− 按住反向' : '+ 按住正向'; button.className = 'compact';
      button.setAttribute('aria-label', `${joint.name} ${direction < 0 ? '负向' : '正向'}点动`);
      button.addEventListener('pointerdown', (event) => {
        if (event.button !== 0) return;
        event.preventDefault();
        act(() => {
          if (!motion.allowed()) throw new Error('请先使能关节');
          motion.stop();
          if (document.getElementById('jog-mode').value === 'inch') {
            motion.target(joint.name, joint.angle + direction * THREE.MathUtils.degToRad(Number(document.getElementById('inch-step').value)));
          } else {
            button.setPointerCapture(event.pointerId);
            motion.state.jog = { name: joint.name, direction };
          }
        });
      });
      const release = () => { motion.state.jog = null; };
      button.addEventListener('pointerup', release); button.addEventListener('pointercancel', release); button.addEventListener('lostpointercapture', release);
      buttons.appendChild(button);
    }
    group.append(label, output, slider, number, go, buttons);
    document.getElementById('joint-controls').appendChild(group);
    controls.push({ joint, output, slider, number, buttons: [...buttons.children, go] });
  }
  const enable = document.getElementById('motion-enable');
  enable.addEventListener('change', () => {
    motion.stop(); motion.state.enabled = enable.checked && !motion.state.estop; enable.checked = motion.state.enabled;
    message.textContent = motion.state.enabled ? '已使能。按住 ＋ / − 点动，或输入角度后点“到位”。' : '已停止，当前未使能。';
  });
  document.getElementById('estop').addEventListener('click', () => {
    motion.stop(); motion.state.estop = true; motion.state.enabled = false; enable.checked = false;
    message.textContent = '急停已锁定；解除后需重新使能';
    cartesianPanel.setStatus('报警：急停已锁定，笛卡尔点动已禁止。', true);
    cartesianPanel.update();
  });
  document.getElementById('release-estop').addEventListener('click', () => {
    motion.state.estop = false;
    message.textContent = '急停已解除，请重新使能';
    cartesianPanel.setStatus('急停已解除，请重新使能后点动。');
  });
  document.getElementById('speed-override').addEventListener('input', (event) => { motion.state.speed = Number(event.target.value); document.getElementById('speed-value').textContent = `${motion.state.speed}%`; });
  document.getElementById('stop-motion').addEventListener('click', motion.stop);
  document.getElementById('reset-joints').addEventListener('click', () => act(() => {
    motion.stop(); for (const joint of api.joints) motion.target(joint.name, 0);
  }));
  const suspend = () => { motion.stop(); motion.state.enabled = false; enable.checked = false; message.textContent = '当前未使能。勾选开关后可继续操作。'; };
  window.addEventListener('blur', suspend);
  document.addEventListener('visibilitychange', () => { if (document.hidden) suspend(); });
  document.getElementById('model-view')?.addEventListener('change', suspend);
  api.setJoint = motion.target;
  return {
    ...motion,
    switchJogMode: jogController.switchJogMode,
    jogStepCartesian: jogController.jogStepCartesian,
    getJogMode: jogController.getJogMode,
    update(delta) {
      motion.update(delta);
      if (motion.state.collision) {
        message.textContent = '已触发防碰撞保护：运动接近地面、基座或工位障碍物，已自动安全拦截。';
        cartesianPanel.setStatus('报警：防碰撞保护已拦截本次运动。', true);
        motion.state.collision = false;
        for (const row of controls) {
          row.slider.value = THREE.MathUtils.radToDeg(row.joint.angle);
          row.number.value = THREE.MathUtils.radToDeg(row.joint.angle).toFixed(1);
        }
      }
      const allowed = motion.allowed() && !motion.state.programActive;
      for (const row of controls) {
        row.output.textContent = `${THREE.MathUtils.radToDeg(row.joint.angle).toFixed(1)}°`;
        if (document.activeElement !== row.slider) row.slider.value = THREE.MathUtils.radToDeg(row.joint.angle);
        row.slider.disabled = row.number.disabled = !allowed;
        row.buttons.forEach((button) => { button.disabled = !allowed; });
      }
      enable.disabled = motion.state.estop;
      document.getElementById('reset-joints').disabled = !allowed;
      document.getElementById('estop').setAttribute('aria-pressed', String(motion.state.estop));
      document.querySelectorAll('.joint-control .button-row').forEach((row) => {
        const inch = document.getElementById('jog-mode').value === 'inch';
        row.children[0].textContent = inch ? '− 一步' : '− 点动';
        row.children[1].textContent = inch ? '+ 一步' : '+ 点动';
      });
      cartesianPanel.update();
    },
  };
}
