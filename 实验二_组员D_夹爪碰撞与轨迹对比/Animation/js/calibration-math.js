import * as THREE from 'three';
import { calibrateUserFrameThreePoints, worldToUserFrame } from './Calibration.js';


export function threePointFrame(origin, xPoint, xyPoint) {
  const points = [origin, xPoint, xyPoint];
  if (points.some((p) => !Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite))) throw new Error('请提供三个完整的有限坐标点');
  const [o, a, b] = points.map((p) => new THREE.Vector3().fromArray(p));
  const inverse = calibrateUserFrameThreePoints(o, a, b), matrix = inverse.clone().invert();
  const x = new THREE.Vector3().setFromMatrixColumn(matrix, 0), y = new THREE.Vector3().setFromMatrixColumn(matrix, 1), z = new THREE.Vector3().setFromMatrixColumn(matrix, 2);
  const local = [o, a, b].map((p) => worldToUserFrame(p, inverse));
  return { matrix: matrix.toArray(), inverse: inverse.toArray(), localPoints: local.map((p) => p.toArray()), determinant: matrix.determinant(),
    orthogonalityError: Math.max(Math.abs(x.dot(y)), Math.abs(x.dot(z)), Math.abs(y.dot(z))),
    roundTripError: Math.max(...[o, a, b].map((p) => p.clone().applyMatrix4(inverse).applyMatrix4(matrix).distanceTo(p))),
    axisError: Math.max(local[0].length(), Math.abs(local[1].y), Math.abs(local[1].z), Math.abs(local[2].z)) };
}

