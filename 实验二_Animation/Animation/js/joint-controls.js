import * as THREE from 'three';
import { createMotion } from './pendant-motion.js';

export function bindJointControls(api, urdfModel) {
  const motion = createMotion(api, urdfModel);
  const message = document.getElementById('motion-status');
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
  });
  document.getElementById('release-estop').addEventListener('click', () => { motion.state.estop = false; message.textContent = '急停已解除，请重新使能'; });
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
  return { ...motion, update(delta) {
    motion.update(delta);
    if (motion.state.collision) {
      message.textContent = '已触发防碰撞保护：运动接近地面、基座或工位障碍物，已自动安全拦截。';
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
  } };
}
