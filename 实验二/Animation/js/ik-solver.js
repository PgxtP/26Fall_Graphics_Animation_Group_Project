import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';

const DEFAULT_POS_TOL = 1e-3;
const DEFAULT_ROT_TOL = 2e-2;
const DEFAULT_MAX_ITER = 80;
const DEFAULT_EPS = 1e-5;
const DEFAULT_ROT_WEIGHT = 0.5;
const DEFAULT_SINGULAR_THRESHOLD = 1e-3;

function toVector3(input) {
  if (!input) return null;
  if (input.isVector3) return input.clone();
  if (Array.isArray(input)) return new THREE.Vector3(input[0] ?? 0, input[1] ?? 0, input[2] ?? 0);
  if (typeof input === 'object' && 'x' in input) return new THREE.Vector3(input.x ?? 0, input.y ?? 0, input.z ?? 0);
  throw new TypeError('targetPos 必须是 THREE.Vector3、{x,y,z} 或 [x,y,z]');
}

function toQuaternion(input) {
  if (input == null) return null;
  if (input.isQuaternion) return input.clone().normalize();
  if (input.isEuler) return new THREE.Quaternion().setFromEuler(input).normalize();
  if (input.isMatrix4) return new THREE.Quaternion().setFromRotationMatrix(input).normalize();
  if (Array.isArray(input) && input.length === 4) {
    return new THREE.Quaternion(input[0], input[1], input[2], input[3]).normalize();
  }
  if (Array.isArray(input) && input.length === 3) {
    return new THREE.Quaternion().setFromEuler(new THREE.Euler(input[0], input[1], input[2])).normalize();
  }
  if (typeof input === 'object' && 'w' in input) {
    return new THREE.Quaternion(input.x ?? 0, input.y ?? 0, input.z ?? 0, input.w ?? 1).normalize();
  }
  if (typeof input === 'object' && 'x' in input) {
    return new THREE.Quaternion().setFromEuler(new THREE.Euler(input.x ?? 0, input.y ?? 0, input.z ?? 0)).normalize();
  }
  throw new TypeError('targetRot 必须是 Quaternion、Euler、Matrix4 或 null');
}

function clampAngles(angles, joints) {
  return angles.map((value, i) => THREE.MathUtils.clamp(value, joints[i].lower, joints[i].upper));
}

function inJointLimits(angles, joints, tol = 1e-5) {
  return angles.every((value, i) => value >= joints[i].lower - tol && value <= joints[i].upper + tol);
}

function jointDistanceSq(a, b) {
  let sum = 0;
  for (let i = 0; i < a.length; i += 1) {
    const d = a[i] - b[i];
    sum += d * d;
  }
  return sum;
}

/**
 * 在给定关节角下计算正运动学 (FK)，返回 TCP 世界位置与世界四元数。
 */
export function evaluateFK(api, angles) {
  for (let i = 0; i < api.joints.length; i += 1) {
    setJointValue(api.joints[i], angles[i]);
  }
  api.tcp.updateWorldMatrix(true, false);
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  api.tcp.getWorldPosition(position);
  api.tcp.getWorldQuaternion(quaternion);
  return { position, quaternion };
}

/**
 * 计算目标姿态与当前姿态的轴角误差向量 (3D) 及标量角度误差 (rad)。
 * q_err = q_target * q_actual^{-1}，取最短弧 (w >= 0)。
 */
function orientationErrorVector(targetQuat, actualQuat) {
  const qErr = targetQuat.clone().multiply(actualQuat.clone().invert()).normalize();
  if (qErr.w < 0) qErr.set(-qErr.x, -qErr.y, -qErr.z, -qErr.w);
  const vNorm = Math.hypot(qErr.x, qErr.y, qErr.z);
  const angle = 2 * Math.atan2(vNorm, Math.max(-1, Math.min(1, qErr.w)));
  if (vNorm < 1e-10) {
    return { vector: new THREE.Vector3(0, 0, 0), angle: 0 };
  }
  const scale = angle / vNorm;
  return {
    vector: new THREE.Vector3(qErr.x * scale, qErr.y * scale, qErr.z * scale),
    angle,
  };
}

/**
 * 高斯消元法（部分主元）求解 m x m 线性方程组 A x = b，同时返回 det(A) 用于计算可操作度。
 */
function solveLinearSystem(matrix, rhs) {
  const m = rhs.length;
  const A = matrix.map((row) => row.slice());
  const b = rhs.slice();
  let detSign = 1;

  for (let col = 0; col < m; col += 1) {
    let pivotRow = col;
    let pivotAbs = Math.abs(A[col][col]);
    for (let row = col + 1; row < m; row += 1) {
      const val = Math.abs(A[row][col]);
      if (val > pivotAbs) {
        pivotAbs = val;
        pivotRow = row;
      }
    }
    if (pivotAbs < 1e-14) {
      return { solution: null, determinant: 0 };
    }
    if (pivotRow !== col) {
      [A[col], A[pivotRow]] = [A[pivotRow], A[col]];
      [b[col], b[pivotRow]] = [b[pivotRow], b[col]];
      detSign = -detSign;
    }
    const pivot = A[col][col];
    for (let row = col + 1; row < m; row += 1) {
      const factor = A[row][col] / pivot;
      if (factor === 0) continue;
      for (let k = col; k < m; k += 1) {
        A[row][k] -= factor * A[col][k];
      }
      b[row] -= factor * b[col];
    }
  }

  let det = detSign;
  for (let i = 0; i < m; i += 1) det *= A[i][i];

  const x = new Array(m).fill(0);
  for (let i = m - 1; i >= 0; i -= 1) {
    let sum = b[i];
    for (let k = i + 1; k < m; k += 1) {
      sum -= A[i][k] * x[k];
    }
    x[i] = sum / A[i][i];
  }
  return { solution: x, determinant: det };
}

/**
 * 计算数值雅可比矩阵 J (m x n) 与可操作度 w = sqrt(det(J * J^T))。
 */
function computeJacobian(api, angles, targetQuat, basePose, rotWeight, eps = DEFAULT_EPS) {
  const n = api.joints.length;
  const m = targetQuat ? 6 : 3;
  const J = Array.from({ length: m }, () => new Array(n).fill(0));

  for (let j = 0; j < n; j += 1) {
    const perturbed = angles.slice();
    perturbed[j] += eps;
    const pose = evaluateFK(api, perturbed);
    const dp = pose.position.clone().sub(basePose.position).multiplyScalar(1 / eps);
    J[0][j] = dp.x;
    J[1][j] = dp.y;
    J[2][j] = dp.z;

    if (targetQuat) {
      const { vector: dRot } = orientationErrorVector(pose.quaternion, basePose.quaternion);
      dRot.multiplyScalar(rotWeight / eps);
      J[3][j] = dRot.x;
      J[4][j] = dRot.y;
      J[5][j] = dRot.z;
    }
  }

  // 计算 J * J^T (m x m)
  const JJT = Array.from({ length: m }, () => new Array(m).fill(0));
  for (let r = 0; r < m; r += 1) {
    for (let c = r; c < m; c += 1) {
      let sum = 0;
      for (let j = 0; j < n; j += 1) sum += J[r][j] * J[c][j];
      JJT[r][c] = sum;
      JJT[c][r] = sum;
    }
  }

  const { determinant } = solveLinearSystem(JJT, new Array(m).fill(0));
  const manipulability = Math.sqrt(Math.max(0, determinant));
  return { J, JJT, manipulability };
}

/**
 * 数值迭代逆解：支持 DLS（阻尼最小二乘，带自适应阻尼与奇异检测）与标准伪逆 pinv。
 */
export function solveIKNumeric(api, targetPos, targetRot = null, currentJoints = null, options = {}) {
  if (!api?.tcp || !api?.joints?.length) {
    throw new Error('solveIKNumeric 需要包含 joints 与 tcp 的机器人对象');
  }
  const n = api.joints.length;
  const targetP = toVector3(targetPos);
  const targetQ = toQuaternion(targetRot);
  const backup = api.joints.map((joint) => joint.angle);
  const seed = Array.isArray(currentJoints) && currentJoints.length === n
    ? clampAngles(currentJoints, api.joints)
    : backup.slice();

  const method = options.method === 'pinv' ? 'pinv' : 'dls';
  const maxIterations = options.maxIterations ?? DEFAULT_MAX_ITER;
  const posTol = options.posTol ?? DEFAULT_POS_TOL;
  const rotTol = options.rotTol ?? DEFAULT_ROT_TOL;
  const rotWeight = options.rotWeight ?? DEFAULT_ROT_WEIGHT;
  const minDamping = options.minDamping ?? 0.005;
  const maxDamping = options.maxDamping ?? 0.15;
  const singularThreshold = options.singularThreshold ?? (targetQ ? 1e-5 : DEFAULT_SINGULAR_THRESHOLD);
  const maxStep = options.maxStep ?? (method === 'pinv' ? 2.0 : 0.25);

  const angles = seed.slice();
  let converged = false;
  let singular = false;
  let posError = Infinity;
  let rotError = 0;
  let error = Infinity;
  let manipulability = 0;
  let dampingUsed = 0;
  let maxStepNorm = 0;
  let iterations = 0;

  try {
    for (let iter = 0; iter < maxIterations; iter += 1) {
      iterations = iter;
      const pose = evaluateFK(api, angles);
      const dp = targetP.clone().sub(pose.position);
      posError = dp.length();

      let residual;
      if (targetQ) {
        const rot = orientationErrorVector(targetQ, pose.quaternion);
        rotError = rot.angle;
        residual = [
          dp.x, dp.y, dp.z,
          rot.vector.x * rotWeight, rot.vector.y * rotWeight, rot.vector.z * rotWeight,
        ];
        error = Math.hypot(posError, rotError * rotWeight);
      } else {
        rotError = 0;
        residual = [dp.x, dp.y, dp.z];
        error = posError;
      }

      if (posError <= posTol && (!targetQ || rotError <= rotTol)) {
        converged = true;
        break;
      }

      const { J, JJT, manipulability: w } = computeJacobian(api, angles, targetQ, pose, rotWeight);
      manipulability = w;

      let lambda = 0;
      if (w < singularThreshold) singular = true;

      if (method === 'dls') {
        if (w >= singularThreshold) {
          lambda = minDamping;
        } else {
          const ratio = 1 - w / singularThreshold;
          lambda = Math.sqrt(minDamping * minDamping + (maxDamping * maxDamping - minDamping * minDamping) * ratio * ratio);
        }
      } else {
        // 标准伪逆（极小正则化避免除零，保留奇异区步长剧增特性以便对比）
        lambda = 1e-5;
      }
      dampingUsed = Math.max(dampingUsed, lambda);

      const m = residual.length;
      const damped = JJT.map((row, r) => row.map((val, c) => val + (r === c ? lambda * lambda : 0)));
      const { solution: y } = solveLinearSystem(damped, residual);
      if (!y) {
        singular = true;
        break;
      }

      const dq = new Array(n).fill(0);
      let stepNormSq = 0;
      for (let j = 0; j < n; j += 1) {
        let sum = 0;
        for (let r = 0; r < m; r += 1) sum += J[r][j] * y[r];
        dq[j] = sum;
        stepNormSq += sum * sum;
      }
      const stepNorm = Math.sqrt(stepNormSq);
      maxStepNorm = Math.max(maxStepNorm, stepNorm);

      // 步长限幅（按向量比例缩放以保持方向）
      const scale = stepNorm > maxStep ? maxStep / stepNorm : 1;
      for (let j = 0; j < n; j += 1) {
        angles[j] = THREE.MathUtils.clamp(angles[j] + dq[j] * scale, api.joints[j].lower, api.joints[j].upper);
      }
    }

    // 最终残差评估
    const finalPose = evaluateFK(api, angles);
    posError = targetP.distanceTo(finalPose.position);
    rotError = targetQ ? orientationErrorVector(targetQ, finalPose.quaternion).angle : 0;
    error = targetQ ? Math.hypot(posError, rotError * rotWeight) : posError;
    converged = posError <= posTol && (!targetQ || rotError <= rotTol);
    const { manipulability: finalW } = computeJacobian(api, angles, targetQ, finalPose, rotWeight);
    manipulability = finalW;
    if (finalW < singularThreshold) singular = true;
  } finally {
    evaluateFK(api, backup);
  }

  return {
    angles,
    converged,
    error,
    posError,
    rotError,
    singular,
    manipulability,
    dampingUsed,
    iterations,
    method,
    maxStepNorm,
    branchesEvaluated: 1,
  };
}

/**
 * 提取机械臂零位几何连杆参数，用于构建多分支几何初解。
 */
function measureArmGeometry(api) {
  const zero = new Array(api.joints.length).fill(0);
  evaluateFK(api, zero);
  const positions = api.joints.map((joint) => {
    const p = new THREE.Vector3();
    joint.node.getWorldPosition(p);
    return p;
  });
  const tcpPos = new THREE.Vector3();
  api.tcp.getWorldPosition(tcpPos);

  const pBase = positions[0] ?? new THREE.Vector3();
  const pShoulder = positions[1] ?? pBase;
  const pElbow = positions[2] ?? pShoulder;
  const L1 = Math.max(0.05, pElbow.distanceTo(pShoulder));
  const L2 = Math.max(0.05, tcpPos.distanceTo(pElbow));
  return { pBase, pShoulder, L1, L2 };
}

/**
 * 生成几何 / 类 Pieper 多分支候选初解（肩部正/反手、肘上/肘下、腕部翻转）。
 */
function generateGeometricBranches(api, targetP, geom, currentAngles) {
  const delta = targetP.clone().sub(geom.pShoulder);
  const planarDist = Math.hypot(delta.x, delta.z);
  const yawBase = Math.atan2(delta.x, delta.z);
  const reach = Math.min(geom.L1 + geom.L2 - 1e-4, Math.max(Math.abs(geom.L1 - geom.L2) + 1e-4, delta.length()));

  const cosElbow = THREE.MathUtils.clamp(
    (reach * reach - geom.L1 * geom.L1 - geom.L2 * geom.L2) / (2 * geom.L1 * geom.L2),
    -1,
    1,
  );
  const elbowMag = Math.acos(cosElbow);
  const elevAngle = Math.atan2(delta.y, Math.max(1e-4, planarDist));
  const shoulderOffset = Math.atan2(geom.L2 * Math.sin(elbowMag), geom.L1 + geom.L2 * Math.cos(elbowMag));

  const j1Candidates = [yawBase, yawBase > 0 ? yawBase - Math.PI : yawBase + Math.PI];
  const elbowConfigs = [
    { j2: elevAngle - shoulderOffset, j3: elbowMag },   // 肘下 / elbow-down
    { j2: elevAngle + shoulderOffset, j3: -elbowMag },  // 肘上 / elbow-up
  ];
  const wristCandidates = [
    currentAngles[3] ?? 0,
    -(currentAngles[3] ?? 0),
  ];

  const seeds = [];
  for (const j1 of j1Candidates) {
    for (const elbow of elbowConfigs) {
      for (const j4 of wristCandidates) {
        const candidate = [
          j1,
          elbow.j2,
          elbow.j3,
          j4,
          currentAngles[4] ?? 0,
        ];
        seeds.push(clampAngles(candidate, api.joints));
      }
    }
  }
  // 额外保留当前姿态作为保底分支
  seeds.push(clampAngles(currentAngles, api.joints));
  return seeds;
}

/**
 * 解析/几何多分支逆解路线：
 * 1. 基于 5-DOF 拓扑几何参数枚举肩/肘/腕多分支候选解；
 * 2. 对各分支执行精确运动学投影并过滤关节限位；
 * 3. 按与当前关节角最近准则 min ||q_sol - q_cur|| 选择最优合法分支。
 */
export function solveIKAnalytic(api, targetPos, targetRot = null, currentJoints = null, options = {}) {
  if (!api?.tcp || !api?.joints?.length) {
    throw new Error('solveIKAnalytic 需要包含 joints 与 tcp 的机器人对象');
  }
  const n = api.joints.length;
  const targetP = toVector3(targetPos);
  const targetQ = toQuaternion(targetRot);
  const backup = api.joints.map((joint) => joint.angle);
  const cur = Array.isArray(currentJoints) && currentJoints.length === n
    ? clampAngles(currentJoints, api.joints)
    : backup.slice();

  let best = null;
  let bestDist = Infinity;
  let branchesEvaluated = 0;

  try {
    const geom = measureArmGeometry(api);
    const branches = generateGeometricBranches(api, targetP, geom, cur);
    const refine = options.refine !== false;
    const validSolutions = [];

    for (const seed of branches) {
      branchesEvaluated += 1;
      const res = refine
        ? solveIKNumeric(api, targetP, targetQ, seed, {
            ...options,
            method: 'dls',
            maxIterations: options.branchMaxIterations ?? 45,
          })
        : (() => {
            const pose = evaluateFK(api, seed);
            const posError = targetP.distanceTo(pose.position);
            const rotError = targetQ ? orientationErrorVector(targetQ, pose.quaternion).angle : 0;
            const rotWeight = options.rotWeight ?? DEFAULT_ROT_WEIGHT;
            const error = targetQ ? Math.hypot(posError, rotError * rotWeight) : posError;
            return {
              angles: seed,
              converged: posError <= (options.posTol ?? DEFAULT_POS_TOL) && (!targetQ || rotError <= (options.rotTol ?? DEFAULT_ROT_TOL)),
              error,
              posError,
              rotError,
              singular: false,
              manipulability: 0,
              dampingUsed: 0,
              iterations: 0,
              method: 'analytic',
              maxStepNorm: 0,
            };
          })();

      if (!inJointLimits(res.angles, api.joints)) continue;
      validSolutions.push(res);
    }

    // 优先在已收敛分支中选择距离当前关节角最近的解
    const convergedList = validSolutions.filter((item) => item.converged);
    const pool = convergedList.length > 0 ? convergedList : validSolutions;

    for (const sol of pool) {
      const dist = convergedList.length > 0
        ? jointDistanceSq(sol.angles, cur)
        : sol.error * 100 + jointDistanceSq(sol.angles, cur);
      if (dist < bestDist) {
        bestDist = dist;
        best = sol;
      }
    }
  } finally {
    evaluateFK(api, backup);
  }

  if (!best) {
    return solveIKNumeric(api, targetP, targetQ, cur, { ...options, method: 'dls' });
  }

  return {
    ...best,
    method: 'analytic',
    branchesEvaluated,
  };
}

/**
 * 统一逆运动学求解入口，供示教器微调、MOVL 轨迹插补与测试基准调用。
 * 同时兼容两种调用形式：
 * - solveIK(api, targetPos, targetRot, currentJoints, options)
 * - solveIK(targetPos, targetRot, currentJoints, options)（自动使用已注册的活动机器人实例）
 */
let activeRobotApi = null;

function isRobotApi(candidate) {
  return Boolean(candidate && typeof candidate === 'object' && Array.isArray(candidate.joints) && candidate.tcp);
}

export function solveIK(apiOrTargetPos, targetPosOrRot = null, targetRotOrJoints = null, currentJointsOrOptions = null, maybeOptions = {}) {
  let api = apiOrTargetPos;
  let targetPos = targetPosOrRot;
  let targetRot = targetRotOrJoints;
  let currentJoints = currentJointsOrOptions;
  let options = maybeOptions;

  if (!isRobotApi(apiOrTargetPos)) {
    api = activeRobotApi ?? globalThis.window?.robotSimulation ?? null;
    targetPos = apiOrTargetPos;
    targetRot = targetPosOrRot;
    currentJoints = targetRotOrJoints;
    options = currentJointsOrOptions ?? {};
  }

  if (!isRobotApi(api)) {
    throw new Error('solveIK 需要传入包含 joints 与 tcp 的机器人对象，或先通过 createIKSolver(api) 绑定实例');
  }
  options = options ?? {};

  const method = options.method ?? 'auto';
  if (method === 'analytic') {
    return solveIKAnalytic(api, targetPos, targetRot, currentJoints, options);
  }
  if (method === 'dls' || method === 'pinv') {
    return solveIKNumeric(api, targetPos, targetRot, currentJoints, options);
  }

  // auto：优先快速数值 DLS 跟踪当前姿态；若未收敛则自动回退到多分支解析搜索
  const numeric = solveIKNumeric(api, targetPos, targetRot, currentJoints, { ...options, method: 'dls' });
  if (numeric.converged) return numeric;
  const analytic = solveIKAnalytic(api, targetPos, targetRot, currentJoints, options);
  return analytic.error <= numeric.error ? analytic : numeric;
}

export function createIKSolver(api) {
  activeRobotApi = api;
  return (targetPos, targetRot = null, currentJoints = null, options = {}) =>
    solveIK(api, targetPos, targetRot, currentJoints, options);
}
