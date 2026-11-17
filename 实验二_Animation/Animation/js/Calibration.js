import * as THREE from 'three';

function vector(value, label) {
  const a = value?.isVector3 ? value.toArray() : value;
  if (!Array.isArray(a) || a.length !== 3 || !a.every(Number.isFinite)) throw new Error(`${label}必须为三个有限数或 THREE.Vector3`);
  return new THREE.Vector3().fromArray(a);
}

// Column-pivoted, re-orthogonalized QR for the three sphere-center unknowns.
// Coordinates are centered/scaled by the caller so the rank test is unit independent.
function solveSphereLeastSquares(rows, right) {
  const columns = [0, 1, 2].map((j) => rows.map((row) => row[j]));
  const order = [0, 1, 2], upper = Array.from({ length: 3 }, () => [0, 0, 0]);
  const projected = [], diagonal = [];
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  const norm = (a) => Math.sqrt(dot(a, a));
  const initialNorm = Math.max(...columns.map(norm));
  for (let k = 0; k < 3; k++) {
    let pivot = k;
    for (let j = k + 1; j < 3; j++) if (norm(columns[j]) > norm(columns[pivot])) pivot = j;
    [columns[k], columns[pivot]] = [columns[pivot], columns[k]];
    [order[k], order[pivot]] = [order[pivot], order[k]];
    for (let i = 0; i < k; i++) [upper[i][k], upper[i][pivot]] = [upper[i][pivot], upper[i][k]];
    const length = norm(columns[k]);
    if (!Number.isFinite(length) || length <= initialNorm * 1e-7) {
      throw new Error('球心无法稳定拟合：采样点重合、共线、共面或接近共面，请增大姿态差异');
    }
    upper[k][k] = length; diagonal.push(length);
    const q = columns[k].map((value) => value / length);
    projected[k] = dot(q, right);
    for (let j = k + 1; j < 3; j++) {
      // A second pass limits loss of orthogonality for narrow pose spreads.
      for (let pass = 0; pass < 2; pass++) {
        const coefficient = dot(q, columns[j]);
        upper[k][j] += coefficient;
        columns[j] = columns[j].map((value, i) => value - coefficient * q[i]);
      }
    }
  }
  const solution = [0, 0, 0], ordered = [0, 0, 0];
  for (let i = 2; i >= 0; i--) {
    let value = projected[i];
    for (let j = i + 1; j < 3; j++) value -= upper[i][j] * ordered[j];
    ordered[i] = value / upper[i][i];
    solution[order[i]] = ordered[i];
  }
  return { solution, condition: Math.max(...diagonal) / Math.min(...diagonal) };
}

const residualSummary = (residuals) => ({
  rms: Math.sqrt(residuals.reduce((sum, value) => sum + value * value / residuals.length, 0)),
  maxResidual: residuals.reduce((maximum, value) => Math.max(maximum, Math.abs(value)), 0),
});

/**
 * Fit a sphere to >=4 flange origins in world coordinates (Vector3 or [x,y,z]).
 * Returns a world-space center, radius, signed radial residuals, RMS/max error,
 * and a QR diagonal-ratio condition proxy (not an exact matrix condition number).
 * Four noncoplanar points always fit a sphere: radial RMS alone cannot verify TCP accuracy.
 */
export function fitSphereCenter(points) {
  if (!Array.isArray(points) || points.length < 4) throw new Error('四点法至少需要4个采样点');
  const values = points.map((point, i) => vector(point, `采样点${i + 1}`));
  const reference = values[0], mean = reference.clone();
  for (const point of values) mean.addScaledVector(point.clone().sub(reference), 1 / values.length);
  const centered = values.map((point) => point.clone().sub(mean));
  const scale = centered.reduce((maximum, point) => Math.max(maximum, point.length()), 0);
  if (!Number.isFinite(scale) || scale <= 0) throw new Error('采样点重合或坐标范围无效，无法拟合球心');
  const normalized = centered.map((point) => point.divideScalar(scale));
  const squared = normalized.map((point) => point.lengthSq());
  const meanSquared = squared.reduce((sum, value) => sum + value / values.length, 0);
  const { solution, condition } = solveSphereLeastSquares(
    normalized.map((point) => point.toArray().map((value) => 2 * value)),
    squared.map((value) => value - meanSquared),
  );
  const center = new THREE.Vector3().fromArray(solution).multiplyScalar(scale).add(mean);
  const distances = values.map((point) => point.distanceTo(center));
  const radius = Math.sqrt(distances.reduce((sum, value) => sum + value * value / values.length, 0));
  const residuals = distances.map((value) => value - radius);
  if (!center.toArray().every(Number.isFinite) || !Number.isFinite(radius)) throw new Error('拟合结果超出有效数值范围');
  return { center, radius, residuals, ...residualSummary(residuals), condition, sampleCount: values.length };
}

function flangePose(value, index) {
  const elements = value?.isMatrix4 ? value.toArray() : value;
  if (!Array.isArray(elements) || elements.length !== 16 || !elements.every(Number.isFinite)) {
    throw new Error(`法兰位姿${index + 1}必须为有限的 Matrix4 或16项列主序数组`);
  }
  if (Math.max(Math.abs(elements[3]), Math.abs(elements[7]), Math.abs(elements[11]), Math.abs(elements[15] - 1)) > 1e-8) {
    throw new Error(`法兰位姿${index + 1}不是仿射变换`);
  }
  const matrix = new THREE.Matrix4().fromArray(elements);
  const axes = [0, 1, 2].map((column) => new THREE.Vector3().setFromMatrixColumn(matrix, column));
  const lengths = axes.map((axis) => axis.length()), scale = (lengths[0] + lengths[1] + lengths[2]) / 3;
  if (!Number.isFinite(scale) || scale <= 0 || lengths.some((length) => Math.abs(length / scale - 1) > 1e-5)) {
    throw new Error(`法兰位姿${index + 1}含奇异或非均匀缩放`);
  }
  axes.forEach((axis) => axis.divideScalar(scale));
  if (Math.max(Math.abs(axes[0].dot(axes[1])), Math.abs(axes[0].dot(axes[2])), Math.abs(axes[1].dot(axes[2]))) > 1e-5 ||
      axes[0].clone().cross(axes[1]).dot(axes[2]) < 1 - 1e-5) {
    throw new Error(`法兰位姿${index + 1}必须为右手刚体位姿（允许统一缩放），不能含剪切或反射`);
  }
  if (!Number.isFinite(matrix.determinant()) || matrix.determinant() === 0) throw new Error(`法兰位姿${index + 1}不可逆`);
  return { matrix, scale };
}

/**
 * Four-point TCP translation calibration from world<-flange Matrix4/column-major arrays.
 * Fit world flange origins, then average inverse(flangePose)*sphereCenter in flange space.
 * TCP orientation is unobservable in a fixed-tip experiment and is intentionally not returned.
 * Uniform scale is supported only when identical across all poses.
 * `residuals`/`rms`/`maxResidual` are world-space contact-consistency errors; inspect these
 * separately from `sphere.residuals`. Inputs are never changed.
 */
export function fitTCPFromPoses(poses) {
  if (!Array.isArray(poses) || poses.length < 4) throw new Error('TCP四点法至少需要4组法兰位姿');
  const checked = poses.map(flangePose), scale = checked[0].scale;
  if (checked.some((pose) => Math.abs(pose.scale / scale - 1) > 1e-5)) throw new Error('各组法兰位姿的缩放必须一致');
  const matrices = checked.map((pose) => pose.matrix);
  const sphere = fitSphereCenter(matrices.map((matrix) => new THREE.Vector3().setFromMatrixPosition(matrix)));
  const localOffsets = matrices.map((matrix) => sphere.center.clone().applyMatrix4(matrix.clone().invert()));
  const offset = new THREE.Vector3();
  for (const point of localOffsets) offset.addScaledVector(point, 1 / poses.length);
  const residuals = matrices.map((matrix) => offset.clone().applyMatrix4(matrix).distanceTo(sphere.center));
  if (!offset.toArray().every(Number.isFinite) || !residuals.every(Number.isFinite)) throw new Error('TCP拟合结果超出有效数值范围');
  return { offset, center: sphere.center.clone(), sphere, localOffsets, residuals,
    ...residualSummary(residuals), sampleCount: poses.length, scale };
}

/** 三点施密特正交化；返回世界→工件矩阵。位置单位与模型一致。 */
export function calibrateUserFrameThreePoints(O, Px, Pxy) {
  const origin = vector(O, 'O'), x = vector(Px, 'Px').sub(origin), plane = vector(Pxy, 'Pxy').sub(origin);
  if (x.length() < 1e-6 || plane.length() < 1e-6) throw new Error('标定点不能重合');
  x.normalize();
  // Gram–Schmidt: subtract projection of the plane vector onto X.
  const y = plane.clone().addScaledVector(x, -plane.dot(x));
  if (y.length() / plane.length() < 1e-4) throw new Error('三点共线或过于接近共线');
  y.normalize();
  const z = x.clone().cross(y).normalize();
  const userToWorld = new THREE.Matrix4().makeBasis(x, y, z).setPosition(origin);
  return userToWorld.invert();
}

/** userMat 必须是世界→工件矩阵，直接左乘，不再求逆。返回新 Vector3。 */
export function worldToUserFrame(pt, userMat) {
  if (!userMat?.isMatrix4 || !userMat.elements.every(Number.isFinite) || Math.abs(userMat.determinant()) < 1e-12) throw new Error('userMat 必须为有效可逆 THREE.Matrix4');
  return vector(pt, 'pt').applyMatrix4(userMat);
}

/** 返回工具→法兰偏移矩阵。rot 可为 [roll,pitch,yaw]（弧度）、Euler 或 Quaternion。 */
export function createTCPOffset(trans, rot) {
  const position = vector(trans, 'trans');
  let quaternion;
  if (rot?.isQuaternion) {
    if (!rot.toArray().every(Number.isFinite) || rot.lengthSq() < 1e-12) throw new Error('工具四元数无效');
    quaternion = rot.clone().normalize();
  } else if (rot?.isEuler) {
    if (![rot.x, rot.y, rot.z].every(Number.isFinite)) throw new Error('工具Euler无效');
    quaternion = new THREE.Quaternion().setFromEuler(rot);
  } else {
    const rpy = vector(rot, 'rot');
    quaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(rpy.x, rpy.y, rpy.z, 'ZYX'));
  }
  return new THREE.Matrix4().compose(position, quaternion, new THREE.Vector3(1, 1, 1));
}
