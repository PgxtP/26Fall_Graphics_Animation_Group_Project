export const MOVL_DEFAULTS = Object.freeze({
  maxSegmentLength: 0.05, // 每段直线最大长度（资产单位），越小越平滑、点数越多
  minSamples: 8, // 最少插补点数（段数 = 点数 - 1）
  maxSamples: 160, // 最多插补点数，防止长距离规划耗时过长
  method: 'dls', // 逆解方法：dls 阻尼最小二乘，热启动（以上一点为初值）最适合连续直线
  posTol: 1e-3, // 位置收敛容差（与 ik-solver 的 DEFAULT_POS_TOL 一致）
  rotTol: 0.25, // 姿态收敛容差（弧度）：兼容五轴机械臂全姿态 SLERP 欠自由度误差，同时防止大幅度偏离
  rotWeight: 0.1, // 五轴位置优先权重：避免不可消去的姿态分量在 DLS 中拖累位置误差超过 posTol
  maxIterations: 120, // 单点最大迭代次数
  maxJointStepDeg: 45, // 相邻插补点的单关节最大变化量，超过视为逆解跳到另一分支
});

const EPSILON = 1e-9;

/** 把 {x,y,z} 或 [x,y,z] 统一成对象；缺失直接报错，避免静默算出 NaN。 */
export function asPosition(value, label = '位姿') {
  if (Array.isArray(value) && value.length >= 3) {
    return { x: Number(value[0]), y: Number(value[1]), z: Number(value[2]) };
  }
  if (value && typeof value === 'object') {
    return { x: Number(value.x), y: Number(value.y), z: Number(value.z) };
  }
  throw new Error(`${label} 缺少位置坐标`);
}

/** 把 {x,y,z,w} 或 [x,y,z,w] 统一成单位四元数；传 null 表示「沿用起点的姿态」。 */
export function asQuaternion(value, fallback = null) {
  let quaternion;
  if (Array.isArray(value) && value.length >= 4) {
    quaternion = { x: Number(value[0]), y: Number(value[1]), z: Number(value[2]), w: Number(value[3]) };
  } else if (value && typeof value === 'object' && 'w' in value) {
    quaternion = { x: Number(value.x), y: Number(value.y), z: Number(value.z), w: Number(value.w) };
  } else if (fallback) {
    return { ...fallback };
  } else {
    throw new Error('位姿缺少姿态四元数');
  }
  const length = Math.hypot(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  if (!(length > EPSILON)) throw new Error('姿态四元数无效（模长为 0）');
  return { x: quaternion.x / length, y: quaternion.y / length, z: quaternion.z / length, w: quaternion.w / length };
}

/** 从 4×4 世界矩阵（THREE.Matrix4.elements，列主序）取出位置与四元数。 */
export function poseFromMatrix(elements) {
  if (!elements || elements.length < 16) throw new Error('世界矩阵无效');
  const m = elements;
  // 列主序下：R00=m[0], R01=m[4], R02=m[8]; R10=m[1], R11=m[5], R12=m[9]; R20=m[2], R21=m[6], R22=m[10]
  const trace = m[0] + m[5] + m[10];
  let x, y, z, w;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s; x = (m[6] - m[9]) / s; y = (m[8] - m[2]) / s; z = (m[1] - m[4]) / s;
  } else if (m[0] > m[5] && m[0] > m[10]) {
    const s = Math.sqrt(1 + m[0] - m[5] - m[10]) * 2;
    w = (m[6] - m[9]) / s; x = 0.25 * s; y = (m[4] + m[1]) / s; z = (m[8] + m[2]) / s;
  } else if (m[5] > m[10]) {
    const s = Math.sqrt(1 + m[5] - m[0] - m[10]) * 2;
    w = (m[8] - m[2]) / s; x = (m[4] + m[1]) / s; y = 0.25 * s; z = (m[9] + m[6]) / s;
  } else {
    const s = Math.sqrt(1 + m[10] - m[0] - m[5]) * 2;
    w = (m[1] - m[4]) / s; x = (m[8] + m[2]) / s; y = (m[9] + m[6]) / s; z = 0.25 * s;
  }
  const length = Math.hypot(x, y, z, w) || 1;
  return {
    position: { x: m[12], y: m[13], z: m[14] },
    quaternion: { x: x / length, y: y / length, z: z / length, w: w / length },
  };
}

/** 球面线性插值（SLERP）：t=0 返回 a，t=1 返回 b，中间保持匀角速度。 */
export function slerp(a, b, t) {
  const qa = asQuaternion(a), qb = asQuaternion(b);
  let cos = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
  let target = qb;
  if (cos < 0) { cos = -cos; target = { x: -qb.x, y: -qb.y, z: -qb.z, w: -qb.w }; } // 取短弧
  if (cos > 0.9995) {
    // 两姿态几乎重合：退化为线性插值再归一化，避免除以 0
    const mix = {
      x: qa.x + (target.x - qa.x) * t, y: qa.y + (target.y - qa.y) * t,
      z: qa.z + (target.z - qa.z) * t, w: qa.w + (target.w - qa.w) * t,
    };
    return asQuaternion(mix);
  }
  const theta = Math.acos(cos), sinTheta = Math.sin(theta);
  const wa = Math.sin((1 - t) * theta) / sinTheta, wb = Math.sin(t * theta) / sinTheta;
  return asQuaternion({
    x: qa.x * wa + target.x * wb, y: qa.y * wa + target.y * wb,
    z: qa.z * wa + target.z * wb, w: qa.w * wa + target.w * wb,
  });
}

const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

/**
 * 规划一条 MOVL 直线。
 *
 * @param {{position:any, quaternion?:any, joints?:number[]}} startPose 起点（位置 + 姿态；可带已知关节角作为初值）
 * @param {{position:any, quaternion?:any}} endPose 终点（姿态留空则沿用起点姿态）
 * @param {number} speed 名义线速度（资产单位/秒），只用于计算理论时长与校验，实际执行受关节限速约束
 * @param {(targetPos:any, targetRot:any, currentJoints:number[]|null, options:any) => any} ikSolver 逆解函数（与 window.robotSimulation.solveIK 同签名）
 * @param {object} [options] 覆盖 MOVL_DEFAULTS 的规划参数
 * @returns {{ok:boolean, samples:Array, errors:Array, ...}} 规划结果
 */
export function planMOVLTrajectory(startPose, endPose, speed, ikSolver, options = {}) {
  const config = { ...MOVL_DEFAULTS, ...(options || {}) };
  if (typeof ikSolver !== 'function') throw new Error('planMOVLTrajectory 需要传入逆解函数 ikSolver');
  if (!Number.isFinite(speed) || speed <= 0) throw new Error('MOVL 速度必须大于 0');
  if (!startPose || !endPose) throw new Error('MOVL 需要起点与终点位姿');

  const p0 = asPosition(startPose.position ?? startPose, 'MOVL 起点');
  const p1 = asPosition(endPose.position ?? endPose, 'MOVL 终点');
  const q0 = asQuaternion(startPose.quaternion, { x: 0, y: 0, z: 0, w: 1 });
  const q1 = asQuaternion(endPose.quaternion, q0);

  const length = distance(p0, p1);
  const sampleCount = length < EPSILON ? config.minSamples
    : clamp(Math.ceil(length / config.maxSegmentLength) + 1, config.minSamples, config.maxSamples);
  const duration = length / speed; // 理论时长（秒），用于报告与进度显示
  const steps = sampleCount - 1;

  let seed = Array.isArray(startPose.joints) && startPose.joints.length
    ? startPose.joints.map(Number)
    : (Array.isArray(config.startJoints) ? config.startJoints.map(Number) : null);

  const samples = [];
  let maxPosError = 0, maxRotError = 0, maxJointStepDeg = 0, singularCount = 0;

  for (let index = 0; index < sampleCount; index += 1) {
    const t = sampleCount === 1 ? 0 : index / steps;
    const position = {
      x: p0.x + (p1.x - p0.x) * t,
      y: p0.y + (p1.y - p0.y) * t,
      z: p0.z + (p1.z - p0.z) * t,
    };
    const quaternion = slerp(q0, q1, t);

    let result;
    try {
      result = ikSolver(position, quaternion, seed, {
        method: config.method,
        posTol: config.posTol,
        rotTol: config.rotTol,
        rotWeight: config.rotWeight,
        maxIterations: config.maxIterations,
      });
    } catch (error) {
      return fail(index, `第 ${index + 1} 个插补点求解异常：${error.message}`, t, position, quaternion);
    }
    if (!result || !Array.isArray(result.angles) || result.angles.length !== 5) {
      return fail(index, `第 ${index + 1} 个插补点逆解返回无效（缺少 5 个关节角）`, t, position, quaternion);
    }

    const posError = Number.isFinite(result.posError) ? result.posError : Infinity;
    const rotError = Number.isFinite(result.rotError) ? result.rotError : Infinity;
    // 不可达的判定只认「未收敛 / 位置超差」；奇异点单独记录，交给第 5 章画曲线。
    if (result.converged !== true || posError > config.posTol * 1.5) {
      return fail(index, `第 ${index + 1} 个插补点不可达（位置误差 ${formatNumber(posError)}，姿态误差 ${formatNumber(rotError)}）`,
        t, position, quaternion);
    }

    const angles = result.angles.map(Number);
    if (samples.length) {
      const previous = samples[samples.length - 1].angles;
      const stepDeg = Math.max(...angles.map((angle, i) => Math.abs(angle - previous[i]))) * 180 / Math.PI;
      if (stepDeg > maxJointStepDeg) maxJointStepDeg = stepDeg;
      if (stepDeg > config.maxJointStepDeg) {
        return fail(index, `第 ${index + 1} 个插补点关节突跳 ${stepDeg.toFixed(1)}°（超过 ${config.maxJointStepDeg}°），疑似逆解切换到其他分支`,
          t, position, quaternion);
      }
    }

    if (result.singular) singularCount += 1;
    maxPosError = Math.max(maxPosError, posError);
    maxRotError = Math.max(maxRotError, rotError);

    samples.push({
      index, t,
      position, quaternion, angles: [...angles],
      posError, rotError,
      singular: result.singular === true,
      manipulability: Number.isFinite(result.manipulability) ? result.manipulability : null,
      dampingUsed: result.dampingUsed ?? null,
      iterations: result.iterations ?? null,
      method: result.method ?? config.method,
      speed: length > EPSILON ? length / duration / steps : 0, // 每段名义线速度
    });
    seed = angles; // 热启动：下一个点用当前解作为初值，保证关节角连续
  }

  return {
    ok: true,
    reason: '',
    failIndex: -1,
    distance: length,
    duration,
    speed,
    sampleCount,
    segmentCount: steps,
    samples,
    maxPosError,
    maxRotError,
    maxJointStepDeg,
    singularCount,
    config,
  };

  function fail(index, reason, t, position, quaternion) {
    return {
      ok: false,
      reason,
      failIndex: index,
      failT: t,
      failPosition: position,
      failQuaternion: quaternion,
      distance: length,
      duration,
      speed,
      sampleCount,
      segmentCount: steps,
      samples: samples.slice(),
      maxPosError,
      maxRotError,
      maxJointStepDeg,
      singularCount,
      config,
    };
  }
}

function formatNumber(value) {
  return Number.isFinite(value) ? value.toExponential(2) : String(value);
}

/** 把规划结果压成一行摘要，供界面状态栏与报告使用。 */
export function summarizeMOVL(result) {
  if (!result) return '尚未规划';
  if (!result.ok) return `MOVL 规划失败：${result.reason}`;
  return `MOVL 直线：${result.distance.toFixed(3)} 资产单位 / ${result.sampleCount} 个插补点`
    + ` / 理论 ${result.duration.toFixed(2)} s / 最大位置误差 ${result.maxPosError.toExponential(2)}`
    + (result.singularCount ? ` / 奇异点 ${result.singularCount} 个` : ' / 全程无奇异点');
}
