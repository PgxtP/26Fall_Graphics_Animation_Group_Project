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
const {
  evaluateFK,
  solveIK,
  solveIKNumeric,
  solveIKAnalytic,
  createIKSolver,
} = await import(url('js/ik-solver.js'));

/**
 * 构建与真实五轴机械臂拓扑一致的测试运动链：
 * J1(Y轴基座偏航) -> J2(X轴肩部俯仰) -> J3(X轴肘部俯仰) -> J4(X轴腕部俯仰) -> J5(Z轴法兰滚转) -> TCP
 */
function buildTestRobot() {
  const robot = new THREE.Object3D();
  const specs = [
    { name: 'J1', offset: [0, 0.35, 0], axis: [0, 1, 0], limitDeg: 120 },
    { name: 'J2', offset: [0, 0.25, 0], axis: [1, 0, 0], limitDeg: 120 },
    { name: 'J3', offset: [0, 0.90, 0], axis: [1, 0, 0], limitDeg: 90 },
    { name: 'J4', offset: [0, 0.75, 0], axis: [1, 0, 0], limitDeg: 120 },
    { name: 'J5', offset: [0, 0.20, 0], axis: [0, 0, 1], limitDeg: 180 },
  ];

  let parent = robot;
  const joints = specs.map((spec) => {
    const node = new THREE.Object3D();
    node.name = spec.name;
    node.position.set(...spec.offset);
    parent.add(node);
    parent = node;
    const limit = THREE.MathUtils.degToRad(spec.limitDeg);
    return {
      name: spec.name,
      node,
      type: 'revolute',
      axis: new THREE.Vector3(...spec.axis).normalize(),
      angle: 0,
      restQuaternion: node.quaternion.clone(),
      lower: -limit,
      upper: limit,
    };
  });

  const tcp = new THREE.Object3D();
  tcp.name = 'tcp';
  tcp.position.set(0, 0.15, 0);
  parent.add(tcp);

  return { robot, joints, tcp };
}

function createSeededRng(seed = 20261113) {
  let state = seed >>> 0;
  return () => {
    state = (1664525 * state + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

test('100组随机可达位姿基准对比：解析多分支路线 vs 数值 DLS 路线', () => {
  const api = buildTestRobot();
  const rng = createSeededRng(42);
  const nSamples = 100;

  let dlsPass = 0;
  let analyticPass = 0;
  let dlsTimeMs = 0;
  let analyticTimeMs = 0;
  let dlsMaxPosErr = 0;
  let analyticMaxPosErr = 0;

  for (let i = 0; i < nSamples; i += 1) {
    // 在安全可达工作空间内随机采样合法关节角
    const gtAngles = api.joints.map((j) => {
      const span = (j.upper - j.lower) * 0.75;
      const mid = (j.lower + j.upper) * 0.5;
      return mid + (rng() - 0.5) * span;
    });
    const targetPose = evaluateFK(api, gtAngles);

    // 构造带扰动的初始猜测（模拟从上一时刻或邻近姿态出发）
    const perturbedSeed = gtAngles.map((a, idx) =>
      THREE.MathUtils.clamp(a + (rng() - 0.5) * 0.35, api.joints[idx].lower, api.joints[idx].upper),
    );

    const t0 = performance.now();
    const dlsRes = solveIKNumeric(api, targetPose.position, targetPose.quaternion, perturbedSeed, {
      method: 'dls',
      posTol: 1e-3,
      rotTol: 2e-2,
      maxIterations: 100,
    });
    dlsTimeMs += performance.now() - t0;
    if (dlsRes.converged) dlsPass += 1;
    dlsMaxPosErr = Math.max(dlsMaxPosErr, dlsRes.posError);

    const t1 = performance.now();
    const analyticRes = solveIKAnalytic(api, targetPose.position, targetPose.quaternion, perturbedSeed, {
      posTol: 1e-3,
      rotTol: 2e-2,
    });
    analyticTimeMs += performance.now() - t1;
    if (analyticRes.converged) analyticPass += 1;
    analyticMaxPosErr = Math.max(analyticMaxPosErr, analyticRes.posError);

    // 验证输出严格在限位内
    analyticRes.angles.forEach((a, idx) => {
      assert.ok(a >= api.joints[idx].lower - 1e-5 && a <= api.joints[idx].upper + 1e-5, '解析解必须满足关节限位');
    });
  }

  console.info('[IK Benchmark 100 poses]', {
    dlsSuccessRate: `${dlsPass}/${nSamples}`,
    dlsAvgMs: (dlsTimeMs / nSamples).toFixed(3),
    dlsMaxPosErr: dlsMaxPosErr.toExponential(2),
    analyticSuccessRate: `${analyticPass}/${nSamples}`,
    analyticAvgMs: (analyticTimeMs / nSamples).toFixed(3),
    analyticMaxPosErr: analyticMaxPosErr.toExponential(2),
  });

  assert.ok(dlsPass >= 95, `数值 DLS 收敛率应 >= 95/100，实际为 ${dlsPass}/100`);
  assert.ok(analyticPass >= 98, `解析多分支收敛率应 >= 98/100，实际为 ${analyticPass}/100`);
});

test('多分支最近解选择准则 min ||q_sol - q_cur|| 与关节限位过滤', () => {
  const api = buildTestRobot();
  // 同一末端位置可通过肘部正角或负角到达
  const elbowDown = [0.2, 0.5, 0.8, 0.2, 0];
  const target = evaluateFK(api, elbowDown);

  const nearElbowDownSeed = [0.2, 0.45, 0.75, 0.15, 0];
  const res = solveIKAnalytic(api, target.position, null, nearElbowDownSeed);

  assert.equal(res.converged, true);
  assert.ok(res.branchesEvaluated >= 4, '应评估多个几何分支');
  assert.ok(res.posError < 1e-3, `位置重投影误差过大的值: ${res.posError}`);
  // 应选到与当前姿态更接近的分支（J3 > 0）
  assert.ok(res.angles[2] > 0, `应选择肘角同号分支，实际 J3=${res.angles[2]}`);
});

test('奇异区压力测试：标准伪逆 pinv 步长剧增 vs 自适应 DLS 平滑收敛', () => {
  const api = buildTestRobot();
  // 完全伸直奇异点附近：J2=0, J3=0, J4=0
  const nearSingularJoints = [0, 0.01, 0.01, 0.01, 0];
  const singularPose = evaluateFK(api, nearSingularJoints);

  // 向奇异不可达方向（正上方微小超限延伸）请求移动 0.05m
  const targetPos = singularPose.position.clone().add(new THREE.Vector3(0.02, 0.05, 0));

  const pinvRes = solveIKNumeric(api, targetPos, null, nearSingularJoints, {
    method: 'pinv',
    maxIterations: 20,
    maxStep: 10,
    singularThreshold: 0.05,
  });

  const dlsRes = solveIKNumeric(api, targetPos, null, nearSingularJoints, {
    method: 'dls',
    maxIterations: 20,
    maxStep: 0.25,
    minDamping: 0.005,
    maxDamping: 0.15,
    singularThreshold: 0.05,
  });

  console.info('[Singularity Stress Test]', {
    pinvMaxStepNorm: pinvRes.maxStepNorm.toFixed(4),
    pinvSingular: pinvRes.singular,
    dlsMaxStepNorm: dlsRes.maxStepNorm.toFixed(4),
    dlsDampingUsed: dlsRes.dampingUsed.toFixed(4),
    dlsSingular: dlsRes.singular,
  });

  assert.equal(dlsRes.singular, true, '奇异区应触发 singular=true 告警');
  assert.ok(dlsRes.dampingUsed > 0.005, '奇异区应自适应增大阻尼 lambda');
  assert.ok(dlsRes.maxStepNorm <= 0.25 + 1e-6, 'DLS 单步关节增量应被平滑限制');
  assert.ok(
    pinvRes.maxStepNorm > dlsRes.maxStepNorm,
    `无阻尼伪逆步长 (${pinvRes.maxStepNorm}) 应明显大于自适应 DLS (${dlsRes.maxStepNorm})`,
  );
});

test('createIKSolver 接口无副作用且不篡改实时关节状态', () => {
  const api = buildTestRobot();
  const initial = [0.3, -0.4, 0.5, 0.2, -0.1];
  evaluateFK(api, initial);
  const solve = createIKSolver(api);

  const target = evaluateFK(api, [-0.3, 0.4, 0.6, -0.2, 0.5]);
  evaluateFK(api, initial); // 恢复当前关节角

  const res = solve(target.position, target.quaternion, initial);
  assert.equal(res.converged, true);
  assert.ok(res.posError < 1e-3);
  api.joints.forEach((joint, idx) => {
    assert.ok(Math.abs(joint.angle - initial[idx]) < 1e-9, 'solveIK 调用后必须恢复原始关节角');
  });

  // 同时验证 4 参直接调用形式 solveIK(targetPos, targetRot, currentJoints, options)
  const fourArgRes = solveIK(target.position, target.quaternion, initial, { method: 'auto' });
  assert.equal(fourArgRes.converged, true);
  assert.ok(fourArgRes.posError < 1e-3);
});

test('真实 joint_map.json 五轴非共面拓扑 50 组随机位姿验证', () => {
  const jointMap = JSON.parse(fs.readFileSync(path.join(root, 'images/joint_map.json'), 'utf8'));
  const robot = new THREE.Object3D();
  let parent = robot;
  const joints = jointMap.joints.map((entry) => {
    const node = new THREE.Object3D();
    node.name = entry.name;
    // Blender Z-up -> glTF Y-up: (x, z, -y)
    const [tx, ty, tz] = entry.translation;
    node.position.set(tx, tz, -ty);
    const [rw, rx, ry, rz] = entry.rfix_quat_wxyz;
    node.quaternion.set(rx, rz, -ry, rw).normalize();
    parent.add(node);
    parent = node;
    return {
      name: entry.name,
      node,
      type: 'revolute',
      axis: new THREE.Vector3(0, 1, 0), // gltfAxis('z') -> (0, 1, 0) in Y-up? Wait: (0,0,1) -> (0,1,0)
      angle: 0,
      restQuaternion: node.quaternion.clone(),
      lower: THREE.MathUtils.degToRad(entry.limit.lower),
      upper: THREE.MathUtils.degToRad(entry.limit.upper),
    };
  });
  const tcp = new THREE.Object3D();
  tcp.name = 'tcp';
  tcp.position.set(0, 0.3, 0);
  parent.add(tcp);

  const api = { robot, joints, tcp };
  const rng = createSeededRng(2026);
  let pass = 0;
  for (let i = 0; i < 50; i += 1) {
    const gt = joints.map((j) => (j.lower + j.upper) * 0.5 + (rng() - 0.5) * (j.upper - j.lower) * 0.7);
    const target = evaluateFK(api, gt);
    const seed = gt.map((a, idx) => THREE.MathUtils.clamp(a + (rng() - 0.5) * 0.3, joints[idx].lower, joints[idx].upper));
    const res = solveIK(api, target.position, target.quaternion, seed, { method: 'auto' });
    if (res.converged) pass += 1;
  }
  console.info(`[Real joint_map.json 50 poses] pass=${pass}/50`);
  assert.ok(pass >= 48, `真实拓扑 50 组随机位姿收敛率应 >= 48/50，实际为 ${pass}/50`);
});

test('工作空间外不可达目标不崩溃、不产生 NaN 且正确返回 converged=false', () => {
  const api = buildTestRobot();
  const res = solveIK(api, [10, 10, 10], null, [0, 0, 0, 0, 0], null);
  assert.equal(res.converged, false);
  assert.ok(Number.isFinite(res.posError) && res.posError > 1);
  assert.ok(res.angles.every((a) => Number.isFinite(a)));
});
