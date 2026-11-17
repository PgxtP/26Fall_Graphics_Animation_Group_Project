/**
 * 第 5 章定量数据生成（无浏览器、纯 Node）。
 *
 * 用法：node lab2/tools/chapter5-data.mjs
 * 读取：<作品根>/js/ik-solver.js、js/movl-planner.js、js/analysis/TrajectoryCharts.js、js/pendant-motion.js
 * 输出：lab2/out/chapter5/data.json、movj.csv、movl.csv、damping.csv、gripper.csv
 *
 * 运动链：与 js/tests/ik.test.mjs 相同的五轴测试链（组长 IK 基准用的同一套关节偏置与限位），
 * 因此这里跑的是工程真实的 IK / MOVL / DLS / 碰撞代码，只有几何尺寸是测试资产。
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = process.env.ANIM_ROOT ?? path.resolve(HERE, '../Animation');
const OUT = process.env.CH5_OUT ?? path.resolve(HERE, '../第五章_图');
fs.mkdirSync(OUT, { recursive: true });

const cache = new Map();
/** 只改写行首的 import/export … from '…'，避免误伤注释里的示例代码。 */
function url(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(ROOT, file), 'utf8').replace(
    /^([ \t]*(?:import|export)[^\n]*?from[ \t]+)'([^']+)'/gm,
    (_, head, name) => `${head}'${url(name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name)))}'`,
  );
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value);
  return value;
}

const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { evaluateFK, createIKSolver, solveIK } = await import(url('js/ik-solver.js'));
const { planMOVLTrajectory, summarizeMOVL } = await import(url('js/movl-planner.js'));
const { straightnessDeviation, jointVelocities, dampingProfile, compareTrajectories } = await import(url('js/analysis/TrajectoryCharts.js'));
const { checkGripperCollision } = await import(url('js/pendant-motion.js'));

const deg2rad = (value) => THREE.MathUtils.degToRad(value);
const rad2deg = (value) => THREE.MathUtils.radToDeg(value);
const round = (value, digits = 6) => (Number.isFinite(value) ? Number(value.toFixed(digits)) : null);

/* ---------------- 1. 五轴测试链（与 js/tests/ik.test.mjs 一致） ---------------- */
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
    const limit = deg2rad(spec.limitDeg);
    return { name: spec.name, node, type: 'revolute', axis: new THREE.Vector3(...spec.axis).normalize(), angle: 0, restQuaternion: node.quaternion.clone(), lower: -limit, upper: limit };
  });
  const tcp = new THREE.Object3D();
  tcp.name = 'tcp';
  tcp.position.set(0, 0.15, 0);
  parent.add(tcp);
  return { robot, joints, tcp, floorY: 0 };
}

const api = buildTestRobot();
const solve = createIKSolver(api);

/* ---------------- 2. 同一对起终点：MOVJ 与 MOVL ---------------- */
const START_DEG = [0, 20, -35, 15, 0];
const END_DEG = [0, 35, -20, 5, 0];
const SPEED_PERCENT = 100;
const MOVL_SPEED = 0.3; // 资产单位/秒
const FRAME_HZ = 60;
const DT = 1 / FRAME_HZ;

const startAngles = START_DEG.map(deg2rad);
const endAngles = END_DEG.map(deg2rad);
const startPose = evaluateFK(api, startAngles);
const endPose = evaluateFK(api, endAngles);

/** MOVJ：完全按 pendant-motion.js update() 的运动律复现（30°/s × 速度倍率，smoothstep 混合）。 */
function simulateMOVJ(from, to) {
  const distance = Math.max(...to.map((value, index) => Math.abs(value - from[index])));
  const step = deg2rad(30 * SPEED_PERCENT / 100) * DT;
  const frames = [];
  let progress = 0;
  let elapsed = 0;
  const limit = Math.ceil((1.5 * distance) / step) + 4;
  for (let index = 0; index < limit; index += 1) {
    progress = distance < 1e-12 ? 1 : Math.min(1, progress + step / (1.5 * distance));
    const blend = progress * progress * (3 - 2 * progress);
    const angles = from.map((value, joint) => value + (to[joint] - value) * blend);
    const pose = evaluateFK(api, angles);
    frames.push({
      t: elapsed,
      angles: angles.map((value) => round(rad2deg(value), 4)),
      position: pose.position.toArray().map((value) => round(value, 6)),
    });
    elapsed += DT;
    if (progress === 1) break;
  }
  return frames;
}

const movjFrames = simulateMOVJ(startAngles, endAngles);
const movjPoints = movjFrames.map((frame) => ({ t: frame.t, x: frame.position[0], y: frame.position[1], z: frame.position[2] }));

// MOVL：工程真实规划器（含逐点 DLS 逆解与奇异计数）
const movlPlan = planMOVLTrajectory(
  { position: startPose.position, quaternion: startPose.quaternion, joints: startAngles },
  { position: endPose.position, quaternion: endPose.quaternion },
  MOVL_SPEED, solve,
);
const movlFrames = movlPlan.samples.map((sample) => {
  const pose = evaluateFK(api, sample.angles);
  return {
    t: movlPlan.duration * sample.t,
    angles: sample.angles.map((value) => round(rad2deg(value), 4)),
    position: pose.position.toArray().map((value) => round(value, 6)),
    dampingUsed: round(sample.dampingUsed, 6),
    manipulability: round(sample.manipulability, 8),
    singular: sample.singular,
  };
});
const movlPoints = movlFrames.map((frame) => ({ t: frame.t, x: frame.position[0], y: frame.position[1], z: frame.position[2] }));

/* ---------------- 3. 指标计算（复用交付模块里的纯函数） ---------------- */
const straightnessMOVJ = straightnessDeviation(movjPoints, startPose.position, endPose.position);
const straightnessMOVL = straightnessDeviation(movlPoints, startPose.position, endPose.position);
const velocityMOVJ = jointVelocities(movjFrames.map((frame) => ({ t: frame.t, joints: frame.angles })));
const velocityMOVL = jointVelocities(movlFrames.map((frame) => ({ t: frame.t, joints: frame.angles })));
const dampingFromPlan = dampingProfile(movlPlan, { gap: 1 });

/* ---------------- 4. 仅位置逆解（3×5 雅可比）的 DLS 阻尼曲线 ----------------
 * MOVL 逐点逆解带姿态约束，雅可比 6×5，J·Jᵀ 是 6×6 且秩最多 5，
 * 因此 det 恒为 0、可操作度恒为 0、自适应阻尼恒为上限。这里额外沿同一条直线
 * 用 targetRot=null 的位置逆解（3×5）取一次 λ 与 w，作为可比较的对照曲线。
 */
const positionOnly = [];
let seed = startAngles.slice();
for (const sample of movlPlan.samples) {
  const result = solveIK(sample.position, null, seed, { method: 'dls' });
  seed = result.angles.slice();
  positionOnly.push({
    s: sample.t,
    lambda: round(result.dampingUsed, 6),
    manipulability: round(result.manipulability, 10),
    singular: result.singular === true,
    posError: round(result.posError, 8),
    iterations: result.iterations ?? null,
  });
}
const dampingPositionOnly = dampingProfile(
  { samples: positionOnly.map((entry) => ({ ...entry, dampingUsed: entry.lambda })) },
  { gap: 1 },
);

/* ---------------- 4b. 自适应阻尼律与工作空间压力测试 ---------------- */
const DAMPING_LAW = { min: 0.005, max: 0.15, threshold: 1e-3 };
const dampingLaw = [];
for (let step = 0; step <= 40; step += 1) {
  const w = (2 * DAMPING_LAW.threshold * step) / 40;
  const ratio = w >= DAMPING_LAW.threshold ? 0 : 1 - w / DAMPING_LAW.threshold;
  dampingLaw.push({
    w: round(w, 8),
    lambda: round(w >= DAMPING_LAW.threshold ? DAMPING_LAW.min : Math.sqrt(DAMPING_LAW.min ** 2 + (DAMPING_LAW.max ** 2 - DAMPING_LAW.min ** 2) * ratio ** 2), 6),
  });
}

const stress = [];
const chordVector = endPose.position.clone().sub(startPose.position);
for (let scale = 0.55; scale <= 1.06; scale += 0.05) {
  const target = startPose.position.clone().addScaledVector(chordVector, scale * 2.2);
  const result = solveIK(target, null, startAngles, { method: 'dls' });
  stress.push({
    scale: round(scale, 3),
    lambda: round(result.dampingUsed, 6),
    manipulability: round(result.manipulability, 10),
    singular: result.singular === true,
    posError: round(result.posError, 6),
    converged: result.converged === true,
  });
}

/* ---------------- 5. 夹爪碰撞：接近台面的最小间距扫描 ---------------- */
function makeBox(width, height, depth, name, position = [0, 0, 0]) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth));
  mesh.name = name;
  mesh.position.fromArray(position);
  return mesh;
}

const blockSize = 0.08;
const gripperRig = new THREE.Object3D();
const flange = new THREE.Object3D();
flange.name = 'link_flange';
gripperRig.add(flange);
const gripper = new THREE.Group();
gripper.name = 'tcp-gripper';
gripper.add(makeBox(blockSize * 1.8, blockSize * 0.22, blockSize * 0.6, 'gripper-palm', [0, blockSize * 0.35, 0]));
gripper.add(makeBox(blockSize * 0.14, blockSize * 0.85, blockSize * 0.45, 'gripper-left', [-blockSize * 0.7, -blockSize * 0.15, 0]));
gripper.add(makeBox(blockSize * 0.14, blockSize * 0.85, blockSize * 0.45, 'gripper-right', [blockSize * 0.7, -blockSize * 0.15, 0]));
flange.add(gripper);
const tableRig = new THREE.Group();
tableRig.name = 'table';
tableRig.add(makeBox(1, 1, 1, 'table1', [2, 0.5, 0]));
const rigScene = new THREE.Scene();
rigScene.add(gripperRig, tableRig);

const clearance = [];
for (let approach = 0; approach <= 0.30 + 1e-9; approach += 0.01) {
  // flange 起始 x=1.2（掌部右缘 1.272），台面左面 x=1.5 → 初始间距 0.228
  flange.position.set(1.2 + approach, 0.5, 0);
  rigScene.updateMatrixWorld(true);
  const result = checkGripperCollision(gripper, [{ object: tableRig, name: '工作台' }], { margin: 0, probeDistance: 0.4 });
  clearance.push({
    approach: round(approach, 3),
    gap: round(result.minDistance, 6),
    colliding: result.colliding,
  });
}
const firstContact = clearance.find((entry) => entry.colliding) ?? null;
const marginSweep = [0, 0.005, 0.01, 0.02, 0.05].map((margin) => {
  flange.position.set(1.42, 0.5, 0); // 掌部右缘 1.492，与台面左面 1.5 留 0.008
  rigScene.updateMatrixWorld(true);
  const result = checkGripperCollision(gripper, [{ object: tableRig, name: '工作台' }], { margin, probeDistance: 0.4 });
  return { margin, colliding: result.colliding, gap: round(result.minDistance, 6), kind: result.hits[0]?.kind ?? null };
});

/* ---------------- 6. 汇总输出 ---------------- */
const summaryRows = compareTrajectories({
  straightness: { MOVJ: straightnessMOVJ, MOVL: straightnessMOVL },
  velocities: { MOVJ: velocityMOVJ, MOVL: velocityMOVL },
  plan: movlPlan,
});

const data = {
  generatedAt: new Date().toISOString(),
  source: { root: ROOT, startDeg: START_DEG, endDeg: END_DEG, speedPercent: SPEED_PERCENT, movlSpeed: MOVL_SPEED, frameHz: FRAME_HZ },
  endpoints: {
    start: startPose.position.toArray().map((value) => round(value, 6)),
    end: endPose.position.toArray().map((value) => round(value, 6)),
    distance: round(startPose.position.distanceTo(endPose.position), 6),
  },
  movj: {
    frames: movjFrames.length,
    duration: round(movjFrames.at(-1).t, 4),
    straightness: {
      max: round(straightnessMOVJ.max), mean: round(straightnessMOVJ.mean), rms: round(straightnessMOVJ.rms),
      chord: round(straightnessMOVJ.chord), pathLength: round(straightnessMOVJ.pathLength),
      percent: round(straightnessMOVJ.percent), detour: round(straightnessMOVJ.detour),
    },
    velocities: {
      max: round(velocityMOVJ.max), meanAbs: round(velocityMOVJ.meanAbs), maxJoint: velocityMOVJ.maxJoint,
      perJoint: velocityMOVJ.series.map((series) => ({ name: series.name, max: round(series.max) })),
    },
    series: movjFrames.map((frame, index) => ({
      t: round(frame.t, 4),
      s: round(straightnessMOVJ.samples[index].s, 6),
      deviation: round(straightnessMOVJ.samples[index].distance),
      joints: frame.angles,
      position: frame.position,
    })),
  },
  movl: {
    ok: movlPlan.ok,
    reason: movlPlan.reason,
    summary: summarizeMOVL(movlPlan),
    sampleCount: movlPlan.sampleCount,
    duration: round(movlPlan.duration, 4),
    maxPosError: round(movlPlan.maxPosError, 8),
    maxRotError: round(movlPlan.maxRotError, 8),
    maxJointStepDeg: round(movlPlan.maxJointStepDeg, 4),
    singularCount: movlPlan.singularCount,
    straightness: {
      max: round(straightnessMOVL.max), mean: round(straightnessMOVL.mean), rms: round(straightnessMOVL.rms),
      chord: round(straightnessMOVL.chord), pathLength: round(straightnessMOVL.pathLength),
      percent: round(straightnessMOVL.percent), detour: round(straightnessMOVL.detour),
    },
    velocities: {
      max: round(velocityMOVL.max), meanAbs: round(velocityMOVL.meanAbs), maxJoint: velocityMOVL.maxJoint,
      perJoint: velocityMOVL.series.map((series) => ({ name: series.name, max: round(series.max) })),
    },
    series: movlFrames.map((frame, index) => ({
      t: round(frame.t, 4),
      s: round(straightnessMOVL.samples[index].s, 6),
      deviation: round(straightnessMOVL.samples[index].distance),
      joints: frame.angles,
      position: frame.position,
      dampingUsed: frame.dampingUsed,
      manipulability: frame.manipulability,
      singular: frame.singular,
    })),
  },
  velocitySeries: {
    MOVJ: velocityMOVJ.series.map((series) => ({ name: series.name, points: series.values.map((point) => [round(point.t, 4), round(point.v, 4)]) })),
    MOVL: velocityMOVL.series.map((series) => ({ name: series.name, points: series.values.map((point) => [round(point.t, 4), round(point.v, 4)]) })),
  },
  damping: {
    law: { ...DAMPING_LAW, points: dampingLaw },
    stress,
    fromPlan: {
      maxLambda: round(dampingFromPlan.maxLambda), meanLambda: round(dampingFromPlan.meanLambda),
      singularCount: dampingFromPlan.singularCount,
      ranges: dampingFromPlan.singularRanges.map((range) => ({ from: range.from, to: range.to, sFrom: round(range.sFrom), sTo: round(range.sTo) })),
      points: dampingFromPlan.samples.map((point) => [round(point.s, 6), round(point.lambda, 6), round(point.manipulability, 8)]),
    },
    positionOnly: {
      maxLambda: round(dampingPositionOnly.maxLambda), meanLambda: round(dampingPositionOnly.meanLambda),
      singularCount: dampingPositionOnly.singularCount,
      minManipulability: round(dampingPositionOnly.minManipulability, 10),
      ranges: dampingPositionOnly.singularRanges.map((range) => ({ from: range.from, to: range.to, sFrom: round(range.sFrom), sTo: round(range.sTo) })),
      points: positionOnly.map((entry) => [round(entry.s, 6), round(entry.lambda, 6), round(entry.manipulability, 10), entry.singular ? 1 : 0]),
    },
  },
  gripper: {
    blockSize,
    firstContact,
    clearance,
    marginSweep,
    triangulated: { gripperTriangles: 36, obstacleTriangles: 12 },
  },
  summaryRows,
};

fs.writeFileSync(path.join(OUT, 'data.json'), JSON.stringify(data, null, 2), 'utf8');

const csv = (name, header, rows) => fs.writeFileSync(path.join(OUT, name), [header, ...rows].join('\n'), 'utf8');
csv('movj.csv', 't,s,deviation,J1,J2,J3,J4,J5,x,y,z',
  data.movj.series.map((row) => [row.t, row.s, row.deviation, ...row.joints, ...row.position].join(',')));
csv('movl.csv', 't,s,deviation,J1,J2,J3,J4,J5,x,y,z,dampingUsed,manipulability,singular',
  data.movl.series.map((row) => [row.t, row.s, row.deviation, ...row.joints, ...row.position, row.dampingUsed, row.manipulability, row.singular ? 1 : 0].join(',')));
csv('damping.csv', 's,lambdaPositionOnly,manipulability,singular,lambdaWithOrientation',
  positionOnly.map((entry, index) => [entry.s, entry.lambda, entry.manipulability, entry.singular ? 1 : 0, data.movl.series[index]?.dampingUsed ?? ''].join(',')));
csv('gripper.csv', 'approach,gap,colliding',
  clearance.map((entry) => [entry.approach, entry.gap, entry.colliding ? 1 : 0].join(',')));

console.log('MOVL:', data.movl.summary);
console.log(`MOVJ 最大偏差 ${data.movj.straightness.max} / MOVL 最大偏差 ${data.movl.straightness.max}`);
console.log(`MOVJ 峰速 ${data.movj.velocities.max} °/s / MOVL 峰速 ${data.movl.velocities.max} °/s`);
console.log(`λ(带姿态) max=${data.damping.fromPlan.maxLambda} 奇异 ${data.damping.fromPlan.singularCount}/${data.movl.sampleCount}`);
console.log(`λ(仅位置) max=${data.damping.positionOnly.maxLambda} 奇异 ${data.damping.positionOnly.singularCount}, w_min=${data.damping.positionOnly.minManipulability}`);
console.log('夹爪首次穿透:', JSON.stringify(firstContact));
console.log('安全间距扫描:', JSON.stringify(marginSweep));
console.log('输出目录:', OUT);
