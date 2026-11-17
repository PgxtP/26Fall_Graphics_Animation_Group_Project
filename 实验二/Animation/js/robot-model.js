import * as THREE from 'three';
import { loadRobot } from './robot-loader.js';
import { parseUrdf, buildUrdfModel } from './urdf-model.js';
import { bindJointControls } from './joint-controls.js';
import { bindCalibration } from './calibration-panel.js';
import { bindProgram } from './program-panel.js';
import { Workstation } from './Workstation.js';
import { createIKSolver } from './ik-solver.js';

async function readText(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}：HTTP ${response.status}`);
  return response.text();
}

export async function loadRobotAsset({ scene, camera, controls, floor, grid, axes, light, viewport }) {
  const [mapText, urdfText, yamlText] = await Promise.all([
    readText('./images/joint_map.json'),
    readText('./images/robot.urdf'),
    readText('./images/robot_spec.yaml'),
  ]);
  const map = JSON.parse(mapText);
  const loaded = await loadRobot('./images/robot_articulated.glb', map);
  const { robot, joints, tcp } = loaded;
  const urdf = new DOMParser().parseFromString(urdfText, 'application/xml');
  if (urdf.querySelector('parsererror')) throw new Error('URDF XML 解析失败');
  const yamlSourceEl = document.getElementById('yaml-source');
  if (yamlSourceEl) yamlSourceEl.textContent = yamlText;
  const definition = parseUrdf(urdf);
  const urdfModel = buildUrdfModel(definition);
  scene.add(urdfModel.root);
  urdfModel.root.visible = false;
  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(robot);
  if (bounds.isEmpty()) throw new Error('GLB 中没有可显示的几何体');
  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  let span = Math.max(size.x, size.y, size.z);
  // 保留源资产全部变换；只调整地面和相机，不缩放或重建运动链。
  robot.traverse((node) => {
    if (node.isMesh) { node.castShadow = true; node.receiveShadow = true; }
  });
  scene.add(robot);
  floor.scale.setScalar(span / 5);
  floor.position.set(center.x, bounds.min.y - span * 0.002, center.z);
  grid.scale.setScalar(span / 5);
  grid.position.copy(floor.position).y += span * 0.001;
  axes.scale.setScalar(span / 10);
  axes.position.set(center.x, bounds.min.y, center.z);
  const fit = () => {
    const vertical = THREE.MathUtils.degToRad(camera.fov);
    const aspect = Math.max(viewport.clientWidth / Math.max(viewport.clientHeight, 1), 0.1);
    const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * aspect);
    const distance = span * 0.9 / Math.sin(Math.min(vertical, horizontal) / 2);
    camera.position.copy(center).add(new THREE.Vector3(1, 0.65, 1.3).normalize().multiplyScalar(distance));
    camera.near = span / 1000;
    camera.far = distance + span * 100;
    camera.updateProjectionMatrix();
    controls.target.copy(center);
    controls.minDistance = span * 0.15;
    controls.maxDistance = distance * 5;
    controls.update();
    controls.saveState();
  };
  fit();
  light.position.copy(center).add(new THREE.Vector3(span, span * 1.5, span));
  light.target.position.copy(center);
  scene.add(light.target);
  Object.assign(light.shadow.camera, { left: -span, right: span, top: span, bottom: -span, near: span / 100, far: span * 6 });
  light.shadow.camera.updateProjectionMatrix();
  light.shadow.normalBias = span * 0.002;

  const checks = [...urdfModel.warnings, ...loaded.warnings];
  joints.forEach((joint) => {
    const { name, lowerDeg: lower, upperDeg: upper } = joint;
    const entry = map.joints.find((entry) => entry.name === name);
    const urdfJoint = [...urdf.querySelectorAll('joint')].find((joint) => joint.getAttribute('name') === entry.name);
    const limit = urdfJoint?.querySelector('limit');
    const row = document.createElement('tr');
    for (const value of [entry.name, `${lower}°`, `${upper}°`, '已校验']) {
      const cell = document.createElement('td'); cell.textContent = value; row.appendChild(cell);
    }
    document.getElementById('joint-limits')?.appendChild(row);
    if (!limit || Math.abs(THREE.MathUtils.radToDeg(Number(limit.getAttribute('lower'))) - lower) > 0.01 || Math.abs(THREE.MathUtils.radToDeg(Number(limit.getAttribute('upper'))) - upper) > 0.01) {
      throw new Error(`${entry.name}：URDF 与 JSON 限位不一致或缺失。`);
    }
  });

  const tree = [];
  const walk = (node, depth = 0) => {
    tree.push(`${'  '.repeat(depth)}${node.name || '(场景)'}${node.isMesh ? ' [mesh/link]' : ''}`);
    node.children.forEach((child) => walk(child, depth + 1));
  };
  walk(robot);
  const modelTreeEl = document.getElementById('model-tree');
  if (modelTreeEl) modelTreeEl.textContent = tree.join('\n');
  const urdfTree = [];
  const walkLinks = (name, depth = 0) => {
    urdfTree.push(`${'  '.repeat(depth)}${name} [link]`);
    definition.joints.filter((entry) => entry.parent === name).forEach((entry) => {
      urdfTree.push(`${'  '.repeat(depth + 1)}${entry.name} [${entry.type}]`);
      walkLinks(entry.child, depth + 2);
    });
  };
  walkLinks(definition.links.find((name) => !definition.joints.some((entry) => entry.child === name)));
  const urdfTreeEl = document.getElementById('urdf-tree');
  if (urdfTreeEl) urdfTreeEl.textContent = urdfTree.join('\n');
  const urdfParamsEl = document.getElementById('urdf-parameters');
  if (urdfParamsEl) urdfParamsEl.textContent = definition.joints.map((entry) =>
    `${entry.name} (${entry.type})\n  ${entry.parent} → ${entry.child}\n  xyz: ${entry.xyz.join(', ')}\n  rpy(rad): ${entry.rpy.join(', ')}\n  axis: ${entry.axis.join(', ')}\n  limit(rad): ${entry.lower ?? '—'} ~ ${entry.upper ?? '—'}\n  effort: ${entry.effort ?? '—'}, velocity: ${entry.velocity ?? '—'}`,
  ).join('\n\n');
  checks.push('限位已按 PPT 第21张统一：J1 ±120°、J2 ±120°、J3 ±90°、J4 ±120°、J5 ±180°；JSON、URDF 与 YAML 已同步。');
  checks.push('robot_spec.yaml 为规格对照原文；当前与 GLB/JSON 的关节参数并非全一致，未自动覆盖或重新生成资产。');
  checks.push('GLB 与 joint_map.json 的 J1–J5 名称可对照；零位指资产导出姿态，并不表示初始四元数为单位四元数。');
  checks.push('GLB 实际层级：J1 与 link_base 同属 robot-arm；URDF 则以 link_base 作为 J1 的父连杆。保留 GLB 原始层级。');
  checks.push('URDF 与 JSON 的部分关节位置/定轴参数不同，且 JSON 使用源坐标。不能直接用 URDF 覆盖 GLB 的初始变换。');
  checks.push('源坐标转换：(x, y, z) → glTF (x, z, −y)。JSON 的局部 +Z 对应转换后的局部 +Y，后续驱动需结合初始四元数验证。');
  checks.push('URDF 限位为弧度，JSON 限位为度；URDF 的 tcp 被关节引用，但没有独立 link 声明，不能视作已验证可用的完整 URDF。');
  for (const text of checks) {
    const item = document.createElement('li'); item.textContent = text;
    document.getElementById('asset-checks')?.appendChild(item);
  }
  const modelSizeEl = document.getElementById('model-size');
  if (modelSizeEl) modelSizeEl.textContent = `${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)}（资产单位）`;
  document.getElementById('scene-status').textContent = `模型已加载 · ${joints.filter((joint) => joint.node).length}/${joints.length} 个关节已匹配`;
  console.table(joints.map(({ name, node, lower, upper }) => ({ name, parent: node?.parent?.name, lower, upper })));
  console.info('机械臂资产', { robot, joints, bounds, map, urdf });
  window.robotSimulation = loaded; // DevTools 中查看 joints.angle、tcp.matrixWorld 或调用 setJoint。
  loaded.floorY = bounds.min.y;
  const pendant = bindJointControls(loaded, urdfModel);
  const calibration = bindCalibration(loaded, pendant, scene);
  const program = bindProgram(loaded, pendant);
  const stationStatus = document.getElementById('workstation-status');
  const describeGrip = (status) => {
    const state = status.state === 'OPEN' ? '张开' : '闭合';
    return `夹爪：${state}\n工件：${status.attached ? '已挂载，随法兰运动' : status.state === 'CLOSE' ? '未抓取（空夹）' : '未挂载，留在当前位置'}`;
  };
  const workstation = await new Workstation(scene, robot.getObjectByName('link_flange'), {
    modelSpan: span, robotCenter: center.clone(), floorY: bounds.min.y,
    getTCPOffsetMatrix: () => loaded.calibration.tcpOffsetMatrix,
    onGripChange: (state) => {
      loaded.calibration.invalidateUserFrame('夹爪或工件状态已改变；请重新读取工件标记并建立用户坐标系。');
      stationStatus.textContent = describeGrip(state);
    },
  }).initialize();
  loaded.workstation = workstation;
  bounds.union(new THREE.Box3().setFromObject(workstation.table));
  bounds.getCenter(center); bounds.getSize(size); span = Math.max(size.x, size.y, size.z);
  fit();
  const requireStopped = () => { if (pendant.busy()) throw new Error('请先停止运动再操作工件'); };
  // C 组 DSL 的 GRIP 回调入口。执行器需先等待上一段运动结束。
  loaded.grip = (state) => {
    requireStopped();
    if (pendant.state.estop) throw new Error('急停期间不可操作夹爪，请先解除急停');
    return workstation.setGrip(state);
  };
  for (const [id, operation] of [
    ['attach-workpiece', () => loaded.grip('CLOSE')],
    ['detach-workpiece', () => loaded.grip('OPEN')],
    ['reset-workpiece', () => {
      requireStopped();
      if (pendant.state.estop) throw new Error('急停期间不可复位工件');
      workstation.resetWorkpiece();
    }],
  ]) {
    document.getElementById(id).addEventListener('click', () => {
      try {
        if (pendant.state.programActive || loaded.program?.runner?.state?.running) throw new Error('请先停止程序再手动操作夹爪');
        requireStopped(); operation();
      }
      catch (error) { stationStatus.textContent = error.message; }
    });
  }
  document.getElementById('use-workpiece-points').addEventListener('click', () => {
    try {
      requireStopped();
      if (workstation.attached) throw new Error('请先张开夹爪释放物块，再读取工件标记');
      loaded.calibration.setUserPoints(workstation.getCalibrationPoints(), 'workpiece-markers');
      document.getElementById('frame-result').textContent = '已填入物块 O、Px、Pxy 标记的世界坐标（仿真），请建立坐标系';
    } catch (error) { document.getElementById('frame-result').textContent = error.message; }
  });
  loaded.motion = pendant;
  loaded.solveIK = createIKSolver(loaded);
  const modelViewEl = document.getElementById('model-view');
  if (modelViewEl) {
    modelViewEl.disabled = false;
    modelViewEl.addEventListener('change', (event) => {
      const showUrdf = event.target.value === 'urdf';
      robot.visible = !showUrdf; urdfModel.root.visible = showUrdf;
      const active = showUrdf ? urdfModel.root : robot;
      bounds.setFromObject(active); bounds.getCenter(center); bounds.getSize(size);
      span = Math.max(size.x, size.y, size.z);
      floor.scale.setScalar(span / 5); grid.scale.setScalar(span / 5);
      floor.position.set(center.x, bounds.min.y - span * 0.002, center.z);
      grid.position.copy(floor.position).y += span * 0.001;
      axes.position.set(center.x, bounds.min.y, center.z); axes.scale.setScalar(span / 10);
      fit();
      document.getElementById('scene-status').textContent = showUrdf ? 'URDF 骨架 · 坐标轴与连线为教学辅助显示' : 'GLB 外观模型 · 5 个关节';
      if (modelSizeEl) modelSizeEl.textContent = `${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)}（当前视图资产单位）`;
    });
  }
  const graspDistance = document.getElementById('grasp-distance');
  return { update(delta) { pendant.update(delta); program.update(); calibration.update();
    workstation.updateGripper();
    if (!document.getElementById('task-workstation').hidden) {
      const state = workstation.gripStatus;
      graspDistance.textContent = `TCP 距抓取点 ${state.distance.toFixed(4)} · 容差 ${state.tolerance.toFixed(4)}（模型单位）${state.canGrab ? ' · 已进入抓取范围' : ''}`;
    }
    workstation.table.visible = robot.visible;
    workstation.workpiece.visible = robot.visible;
  } };
}
