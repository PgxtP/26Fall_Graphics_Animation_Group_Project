import * as THREE from 'three';

/** 固定种子确保报告数据可复现；不保存/更改程序、工件或图表。 */
export function verifyRobotFK(robot, kinematics, toolOffsetMatrix) {
  let seed = 20261023;
  const random = () => { seed = (Math.imul(1664525, seed) + 1013904223) >>> 0; return seed / 4294967296; };
  const cases = [{ name: 'zero', angles: [0, 0, 0, 0, 0, 0] }];
  for (let joint = 0; joint < 6; joint++) {
    for (const sign of [-1, 1]) {
      const angles = [0, 0, 0, 0, 0, 0];
      angles[joint] = sign * Math.PI / 2;
      cases.push({ name: 'J' + (joint + 1) + (sign < 0 ? '-90' : '+90'), angles });
    }
  }
  for (let i = 0; i < 100; i++) {
    cases.push({ name: 'random-' + String(i + 1).padStart(3, '0'),
      angles: robot.jointLimits.map(l => l.min + random() * (l.max - l.min)) });
  }
  const original = robot.getJointAngles();
  const rows = [];
  try {
    for (const test of cases) {
      const result = robot.setJointAngles(test.angles);
      if (!result.success || result.errorMsg) throw new Error('验证用例超出当前限位：' + test.name);
      const actual = robot.getTCPPose(toolOffsetMatrix).matrixWorld;
      rows.push({
        name: test.name, anglesRad: test.angles,
        anglesDeg: test.angles.map(THREE.MathUtils.radToDeg),
        position: new THREE.Vector3().setFromMatrixPosition(actual).toArray(),
        matrixWorld: actual.toArray(),
        ...kinematics.verifyFK(test.angles, actual, toolOffsetMatrix),
      });
    }
  } finally { robot.setJointAngles(original); }
  return {
    seed: 20261023, tolerance: 1e-6, total: rows.length, passed: rows.filter(r => r.isPassed).length,
    maxPositionError: Math.max(...rows.map(r => r.positionError)),
    maxMatrixDiff: Math.max(...rows.map(r => r.matrixMaxDiff)), rows,
  };
}
