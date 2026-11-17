import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';

const SEED_DEG = [20, -20, 25, 15, 0];
const SEGMENTS = 24;
const RADIUS = 0.6;
const TOLERANCE = 0.005;
const EPSILON = 1e-4;
const DAMPING = 0.08;

export function createCartesianScan(api) {
  if (!api?.tcp || api.joints?.length !== 5) throw new Error('360° 扫描需要五关节模型与 TCP');
  const backup = api.joints.map((joint) => joint.angle);
  const angles = SEED_DEG.map(THREE.MathUtils.degToRad);
  const position = new THREE.Vector3();
  const sample = (values) => {
    values.forEach((value, index) => setJointValue(api.joints[index], value));
    api.tcp.updateWorldMatrix(true, false);
    return api.tcp.getWorldPosition(position).clone();
  };
  try {
    const center = sample(angles);
    const poses = [];
    let maxWaypointError = 0;
    for (let index = 0; index <= SEGMENTS; index += 1) {
      const phase = index / SEGMENTS * Math.PI * 2;
      const target = center.clone().add(new THREE.Vector3(RADIUS * Math.cos(phase), 0, RADIUS * Math.sin(phase)));
      angles[4] = -Math.PI + phase;
      let error = Infinity;
      for (let iteration = 0; iteration < 120; iteration += 1) {
        const actual = sample(angles);
        const offset = target.clone().sub(actual);
        error = offset.length();
        if (error <= TOLERANCE) break;
        const columns = [];
        for (let joint = 0; joint < 4; joint += 1) {
          const perturbed = [...angles];
          perturbed[joint] += EPSILON;
          columns.push(sample(perturbed).sub(actual).multiplyScalar(1 / EPSILON));
        }
        const jj = new THREE.Matrix3();
        const e = jj.elements;
        for (let row = 0; row < 3; row += 1) {
          for (let col = 0; col < 3; col += 1) {
            e[col * 3 + row] = columns.reduce((sum, v) => sum + v.getComponent(row) * v.getComponent(col), 0)
              + (row === col ? DAMPING * DAMPING : 0);
          }
        }
        jj.invert();
        const step = offset.applyMatrix3(jj);
        for (let joint = 0; joint < 4; joint += 1) {
          const change = THREE.MathUtils.clamp(columns[joint].dot(step), -0.08, 0.08);
          angles[joint] = THREE.MathUtils.clamp(angles[joint] + change, api.joints[joint].lower, api.joints[joint].upper);
        }
      }
      error = sample(angles).distanceTo(target);
      if (error > TOLERANCE) throw new Error(`扫描点 P${index + 1} 不可达（TCP 误差 ${error.toFixed(3)}）`);
      maxWaypointError = Math.max(maxWaypointError, error);
      poses.push({
        name: `P${index + 1}`,
        angles: [...angles],
        tcpWorld: api.tcp.matrixWorld.toArray(),
        targetWorld: target.toArray()
      });
    }
    return { center: center.toArray(), radius: RADIUS, segments: SEGMENTS, maxWaypointError, poses };
  } finally {
    backup.forEach((value, index) => setJointValue(api.joints[index], value));
    api.robot.updateMatrixWorld(true);
  }
}

