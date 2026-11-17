import * as THREE from 'three';

function vector(value, label) {
  const a = value?.isVector3 ? value.toArray() : value;
  if (!Array.isArray(a) || a.length !== 3 || !a.every(Number.isFinite)) throw new Error(`${label}必须为三个有限数或 THREE.Vector3`);
  return new THREE.Vector3().fromArray(a);
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
