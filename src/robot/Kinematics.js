import * as THREE from 'three';
import { ROBOT_CONFIG } from './robot3Config.js';

export function validateAngles(angles) {
  if (!Array.isArray(angles) || angles.length !== 6 || !Array.from(angles).every(Number.isFinite)) {
    throw new TypeError('关节角必须是包含 6 个有限数值的弧度数组');
  }
}

export function validateRigidOffset(matrix) {
  if (!matrix?.isMatrix4 || !Array.from(matrix.elements).every(Number.isFinite)) throw new TypeError('工具偏置必须是有限的 THREE.Matrix4');
  const e = matrix.elements;
  const cols = [new THREE.Vector3(e[0], e[1], e[2]), new THREE.Vector3(e[4], e[5], e[6]), new THREE.Vector3(e[8], e[9], e[10])];
  if (cols.some(v => Math.abs(v.lengthSq() - 1) > 1e-6) ||
      Math.abs(cols[0].dot(cols[1])) > 1e-6 || Math.abs(cols[0].dot(cols[2])) > 1e-6 ||
      Math.abs(cols[1].dot(cols[2])) > 1e-6 || Math.abs(matrix.determinant() - 1) > 1e-6 ||
      Math.abs(e[3]) + Math.abs(e[7]) + Math.abs(e[11]) + Math.abs(e[15] - 1) > 1e-6) {
    throw new TypeError('工具偏置只允许刚体旋转和平移，不能包含缩放或反射');
  }
}

// 显式展开旋转矩阵，不调用场景树、Object3D 或 Quaternion 的组合算法。
function restRotation([x, y, z, w]) {
  return [
    1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w),
    2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w),
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y),
  ];
}

function axisRotation([x, y, z], angle) {
  const c = Math.cos(angle), s = Math.sin(angle), v = 1 - c;
  return [
    c+x*x*v, x*y*v-z*s, x*z*v+y*s,
    y*x*v+z*s, c+y*y*v, y*z*v-x*s,
    z*x*v-y*s, z*y*v+x*s, c+z*z*v,
  ];
}

/** 静态函数使用课程模型的默认工位；移动模型后使用实例及 getKinematicConfig()。 */
export class Kinematics {
  constructor(config = ROBOT_CONFIG) {
    this.baseMatrix = new THREE.Matrix4().fromArray(config.baseMatrix);
    this.chain = config.chain.map(node => ({ ...structuredClone(node), restRotation: restRotation(node.quaternion) }));
  }

  forwardKinematics(anglesRad, toolOffsetMatrix) {
    validateAngles(anglesRad);
    const result = this.baseMatrix.clone();
    for (const node of this.chain) {
      const fixed = node.restRotation;
      let r = fixed;
      if (node.jointIndex >= 0) {
        const a = axisRotation(node.axis, anglesRad[node.jointIndex]);
        r = new Array(9);
        for (let row = 0; row < 3; row++) {
          for (let col = 0; col < 3; col++) {
            r[row*3+col] = a[row*3]*fixed[col] + a[row*3+1]*fixed[3+col] + a[row*3+2]*fixed[6+col];
          }
        }
      }
      const [sx, sy, sz] = node.scale;
      const [x, y, z] = node.translation;
      result.multiply(new THREE.Matrix4().set(
        r[0]*sx, r[1]*sy, r[2]*sz, x,
        r[3]*sx, r[4]*sy, r[5]*sz, y,
        r[6]*sx, r[7]*sy, r[8]*sz, z,
        0, 0, 0, 1,
      ));
    }
    if (toolOffsetMatrix) { validateRigidOffset(toolOffsetMatrix); result.multiply(toolOffsetMatrix); }
    return result;
  }

  verifyFK(testAngles, actualThreeMatrix, toolOffsetMatrix) {
    const expected = this.forwardKinematics(testAngles, toolOffsetMatrix);
    if (!actualThreeMatrix?.isMatrix4 || !Array.from(actualThreeMatrix.elements).every(Number.isFinite)) throw new TypeError('对拍矩阵必须是有限的 THREE.Matrix4');
    const positionError = new THREE.Vector3().setFromMatrixPosition(expected).distanceTo(new THREE.Vector3().setFromMatrixPosition(actualThreeMatrix));
    const matrixMaxDiff = Math.max(...expected.elements.map((value, i) => Math.abs(value - actualThreeMatrix.elements[i])));
    return { positionError, matrixMaxDiff, isPassed: positionError < 1e-6 && matrixMaxDiff < 1e-6 };
  }

  static forwardKinematics(anglesRad, toolOffsetMatrix) { return defaultKinematics.forwardKinematics(anglesRad, toolOffsetMatrix); }
  static verifyFK(testAngles, actualThreeMatrix, toolOffsetMatrix) { return defaultKinematics.verifyFK(testAngles, actualThreeMatrix, toolOffsetMatrix); }
}
const defaultKinematics = new Kinematics();
