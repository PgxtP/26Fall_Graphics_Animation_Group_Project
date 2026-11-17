import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const cache = new Map();
function url(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8').replace(
    /from '([^']+)'/g,
    (_, name) => `from '${url(name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name)))}'`,
  );
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value);
  return value;
}

const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { JOG_MODES, createJogController, getTCPPoseInJogFrame } = await import(url('js/joint-controls.js'));

function buildApi() {
  const robot = new THREE.Object3D();
  const base = new THREE.Object3D();
  base.name = 'link_base';
  robot.add(base);
  const tcp = new THREE.Object3D();
  tcp.name = 'tcp';
  tcp.position.set(0.4, 0.8, -0.2);
  base.add(tcp);
  const joints = Array.from({ length: 5 }, (_, index) => ({
    name: `J${index + 1}`,
    angle: index * 0.01,
    lower: -1,
    upper: 1,
  }));
  robot.updateMatrixWorld(true);
  return { robot, base, tcp, joints };
}

function assertVector(actual, expected, tolerance = 1e-10) {
  assert.ok(actual.distanceTo(expected) <= tolerance, `期望 ${expected.toArray()}，实际 ${actual.toArray()}`);
}

function successResult(angles = [0.1, 0.2, 0.3, 0.4, 0.5]) {
  return { angles, converged: true, singular: false, error: 0 };
}

test('支持 Joint、Base、Tool、User 四种模式，非法模式被拒绝', () => {
  const api = buildApi();
  let stopped = 0;
  const controller = createJogController(api, { stopMotion: () => { stopped += 1; } });
  assert.deepEqual(JOG_MODES, ['Joint', 'Base', 'Tool', 'User']);
  assert.equal(controller.getJogMode(), 'Joint');
  assert.equal(controller.switchJogMode('base'), 'Base');
  assert.equal(controller.switchJogMode('TOOL'), 'Tool');
  assert.equal(controller.switchJogMode('User'), 'User');
  assert.equal(stopped, 3);
  assert.throws(() => controller.switchJogMode('World'), /不支持的点动模式/);
});

test('Joint 模式不执行笛卡尔点动', () => {
  const controller = createJogController(buildApi());
  assert.throws(() => controller.jogStepCartesian('X', 0.01), /Joint 模式/);
});

test('Base 模式按 link_base 轴向计算目标，并把完整五关节数组交给运动层', () => {
  const api = buildApi();
  api.base.rotation.y = Math.PI / 2;
  api.robot.updateMatrixWorld(true);
  const start = api.tcp.getWorldPosition(new THREE.Vector3());
  let receivedTarget = null;
  let applied = null;
  const controller = createJogController(api, {
    canMove: () => true,
    solveIK(targetPosition, targetRotation, currentJoints, options) {
      receivedTarget = { targetPosition, targetRotation, currentJoints, options };
      return successResult();
    },
    applyAngles: (angles) => { applied = angles; },
  });
  controller.switchJogMode('Base');
  const result = controller.jogStepCartesian('X', 0.25);
  assertVector(receivedTarget.targetPosition, start.clone().add(new THREE.Vector3(0, 0, -0.25)));
  assert.equal(receivedTarget.targetRotation, null);
  assert.deepEqual(receivedTarget.currentJoints, api.joints.map((joint) => joint.angle));
  assert.deepEqual(receivedTarget.options, { method: 'auto' });
  assert.deepEqual(applied, [0.1, 0.2, 0.3, 0.4, 0.5]);
  assert.deepEqual(result.angles, applied);
});

test('Tool 模式按 TCP 当前姿态换算轴向', () => {
  const api = buildApi();
  api.tcp.rotation.z = Math.PI / 2;
  api.robot.updateMatrixWorld(true);
  const start = api.tcp.getWorldPosition(new THREE.Vector3());
  let target = null;
  const controller = createJogController(api, {
    solveIK: (targetPosition) => { target = targetPosition; return successResult(); },
  });
  controller.switchJogMode('Tool');
  controller.jogStepCartesian('X', 0.1);
  assertVector(target, start.clone().add(new THREE.Vector3(0, 0.1, 0)));
});

test('控制器创建后仍会调用组长稍后挂载到 api.solveIK 的正式 IK 接口', () => {
  const api = buildApi();
  let solverCalls = 0;
  let applied = null;
  const controller = createJogController(api, {
    applyAngles: (angles) => { applied = angles; },
  });
  api.solveIK = (targetPosition, targetRotation, currentJoints, options) => {
    solverCalls += 1;
    assert.equal(targetPosition.isVector3, true);
    assert.equal(targetRotation, null);
    assert.deepEqual(currentJoints, api.joints.map((joint) => joint.angle));
    assert.deepEqual(options, { method: 'auto' });
    return successResult();
  };
  controller.switchJogMode('Base');
  controller.jogStepCartesian('Z', 0.05);
  assert.equal(solverCalls, 1);
  assert.deepEqual(applied, [0.1, 0.2, 0.3, 0.4, 0.5]);
});

test('User 模式使用组员 B 提供的 world-to-user 矩阵换算轴向', () => {
  const api = buildApi();
  const userToWorld = new THREE.Matrix4().makeRotationZ(Math.PI / 2);
  api.calibration = { userMatrix: userToWorld.clone().invert() };
  const start = api.tcp.getWorldPosition(new THREE.Vector3());
  let target = null;
  const controller = createJogController(api, {
    solveIK: (targetPosition) => { target = targetPosition; return successResult(); },
  });
  controller.switchJogMode('User');
  controller.jogStepCartesian('X', -0.2);
  assertVector(target, start.clone().add(new THREE.Vector3(0, -0.2, 0)));
});

test('HUD 按 Base、Tool、User 坐标系实时换算 TCP 相对位姿', () => {
  const api = buildApi();
  api.base.position.set(2, 3, 4);
  api.base.rotation.y = Math.PI / 3;
  api.tcp.rotation.z = Math.PI / 6;
  api.robot.updateMatrixWorld(true);

  const basePose = getTCPPoseInJogFrame(api, 'Base');
  assertVector(basePose.position, api.tcp.position);
  assert.ok(Math.abs(basePose.rpy.z - Math.PI / 6) < 1e-10);

  const jointPose = getTCPPoseInJogFrame(api, 'Joint');
  assertVector(jointPose.position, basePose.position);
  assert.match(jointPose.frameLabel, /Joint/);

  const toolPose = getTCPPoseInJogFrame(api, 'Tool');
  assertVector(toolPose.position, new THREE.Vector3());
  assert.ok(Math.abs(toolPose.rpy.x) < 1e-10 && Math.abs(toolPose.rpy.y) < 1e-10 && Math.abs(toolPose.rpy.z) < 1e-10);

  const userToWorld = new THREE.Matrix4().makeRotationZ(Math.PI / 2).setPosition(1, 2, 3);
  api.calibration = { userMatrix: userToWorld.clone().invert() };
  const userPose = getTCPPoseInJogFrame(api, 'User');
  const expectedUserPosition = api.tcp.getWorldPosition(new THREE.Vector3()).applyMatrix4(api.calibration.userMatrix);
  assertVector(userPose.position, expectedUserPosition);
});

test('未建立用户坐标系、未使能或步长无效时直接拦截', () => {
  const api = buildApi();
  let solverCalls = 0;
  const disabled = createJogController(api, {
    canMove: () => false,
    solveIK: () => { solverCalls += 1; return successResult(); },
  });
  disabled.switchJogMode('Base');
  assert.throws(() => disabled.jogStepCartesian('X', 0.1), /使能/);
  assert.equal(solverCalls, 0);

  const controller = createJogController(api);
  controller.switchJogMode('User');
  assert.throws(() => controller.jogStepCartesian('X', 0.1), /尚未建立用户坐标系/);
  controller.switchJogMode('Base');
  assert.throws(() => controller.jogStepCartesian('Q', 0.1), /不支持的笛卡尔轴/);
  assert.throws(() => controller.jogStepCartesian('X', 0), /步长/);
});

test('IK 未收敛、奇异、格式错误或超限时不下发运动', () => {
  const cases = [
    [{ angles: [0, 0, 0, 0, 0], converged: false, singular: false }, /未收敛/],
    [{ angles: [0, 0, 0, 0, 0], converged: true, singular: true }, /奇异位形/],
    [{ angles: [0, 0], converged: true, singular: false }, /数量或数值无效/],
    [{ angles: [0, 0, Number.NaN, 0, 0], converged: true, singular: false }, /数量或数值无效/],
    [{ angles: [0, 0, 0, 0, 2], converged: true, singular: false }, /超出关节限位/],
  ];

  for (const [ikResult, expectedError] of cases) {
    const api = buildApi();
    let applied = 0;
    const controller = createJogController(api, {
      solveIK: () => ikResult,
      applyAngles: () => { applied += 1; },
    });
    controller.switchJogMode('Base');
    assert.throws(() => controller.jogStepCartesian('Z', 0.1), expectedError);
    assert.equal(applied, 0);
  }
});
