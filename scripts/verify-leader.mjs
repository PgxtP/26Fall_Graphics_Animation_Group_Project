import assert from 'node:assert/strict';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RobotModel } from '../src/robot/RobotModel.js';
import { Kinematics } from '../src/robot/Kinematics.js';
import { ROBOT_CONFIG } from '../src/robot/robot3Config.js';
import { verifyRobotFK } from '../src/robot/verifyRobotFK.js';

const bytes = await readFile(new URL('../public/models/robot3/robot_articulated.glb', import.meta.url));
const { scene } = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
const placement = new THREE.Group();
placement.position.fromArray(ROBOT_CONFIG.placement.position);
placement.quaternion.fromArray(ROBOT_CONFIG.placement.quaternion);
placement.add(scene);
const robot = new RobotModel(scene);
const fk = new Kinematics(robot.getKinematicConfig());
const suite = verifyRobotFK(robot, fk);
assert.equal(suite.passed, 113);
for (const row of suite.rows) {
  assert(Kinematics.verifyFK(row.anglesRad, new THREE.Matrix4().fromArray(row.matrixWorld)).isPassed, '默认静态 FK 与实际工位必须一致');
}
const checks = [];
function check(name, fn) { fn(); checks.push({ name, passed: true }); }
check('独立 NumPy 零位数值基准', () => {
  const expected = new THREE.Vector3(9.086854903037098, 16.931712582362895, -0.0012093600936280424);
  assert(robot.getTCPPose().position.distanceTo(expected) < 1e-10);
});
check('J1 正向 90° 符合基座 +Y 右手旋转', () => {
  const p0 = robot.getTCPPose().position.clone();
  const pivot = robot.joints[0].getWorldPosition(new THREE.Vector3());
  const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(robot.joints[0].parent.getWorldQuaternion(new THREE.Quaternion()));
  assert(axis.dot(new THREE.Vector3(0, 1, 0)) > 1 - 1e-12);
  const expected = p0.sub(pivot).applyAxisAngle(axis, Math.PI / 2).add(pivot);
  robot.setJointAngles([Math.PI / 2, 0, 0, 0, 0, 0]);
  assert(robot.getTCPPose().position.distanceTo(expected) < 1e-10);
  robot.setJointAngles([0, 0, 0, 0, 0, 0]);
});
check('12 个上下软限位与返回数组隔离', () => {
  for (const sign of [-1, 1]) {
    const r = robot.setJointAngles(Array(6).fill(sign * 100));
    assert(r.success);
    assert(r.errorMsg);
    assert.deepEqual(robot.getJointAngles(), robot.jointLimits.map(l => sign < 0 ? l.min : l.max));
    r.clampedAngles[0] = 123;
    assert.notEqual(robot.getJointAngles()[0], 123);
  }
});
check('非法输入原子拒绝，不污染现有关节状态', () => {
  const before = robot.getJointAngles();
  for (const invalid of [null, [], Array(6), [1, 2], [0, 0, 0, 0, 0, NaN], [0, 0, 0, Infinity, 0, 0], ['1', 0, 0, 0, 0, 0]]) {
    assert.equal(robot.setJointAngles(invalid).success, false);
    assert.deepEqual(robot.getJointAngles(), before);
    assert.throws(() => fk.forwardKinematics(invalid));
  }
});
check('重复赋值 1000 次不累积零位旋转误差', () => {
  const angles = [0.3, -0.7, 0.5, 1.1, -0.8, 0.2];
  robot.setJointAngles(angles);
  const before = robot.getTCPPose().matrixWorld.clone();
  for (let i = 0; i < 1000; i++) robot.setJointAngles(angles);
  assert(fk.verifyFK(angles, before).isPassed);
  assert.deepEqual(robot.getTCPPose().matrixWorld.toArray(), before.toArray());
});
check('工具偏置正确右乘，位置/旋转/四元数一致', () => {
  const offset = new THREE.Matrix4().makeRotationZ(0.4).setPosition(0.2, -0.1, 0.3);
  const pose = robot.getTCPPose(offset);
  const expected = robot.flange.matrixWorld.clone().multiply(offset);
  assert(pose.matrixWorld.equals(expected));
  assert(fk.verifyFK(robot.getJointAngles(), pose.matrixWorld, offset).isPassed);
  assert(Math.abs(pose.quaternion.length() - 1) < 1e-12);
  const r = pose.rotationRPY;
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(r.roll, r.pitch, r.yaw, 'ZYX'));
  assert(q.angleTo(pose.quaternion) < 1e-7);
  const changed = pose.position.clone().sub(robot.getTCPPose().position).length();
  assert(Math.abs(changed - Math.sqrt(0.14)) < 1e-10);
  for (const invalid of [new THREE.Matrix4().makeScale(2, 1, 1), new THREE.Matrix4().makeScale(-1, 1, 1), {}]) {
    assert.throws(() => robot.getTCPPose(invalid));
  }
});
check('FK 对错误矩阵真实失败，不能虚报通过', () => {
  const actual = robot.getTCPPose().matrixWorld.clone();
  actual.elements[12] += 0.001;
  const result = fk.verifyFK(robot.getJointAngles(), actual);
  assert.equal(result.isPassed, false);
  assert(result.positionError > 0.0009);
});
check('纯数学 FK 调用不改变场景与当前关节值', () => {
  const before = robot.getJointAngles();
  const matrices = robot.joints.map(n => n.matrixWorld.toArray());
  for (let i = 0; i < 1000; i++) fk.forwardKinematics([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
  assert.deepEqual(robot.getJointAngles(), before);
  assert.deepEqual(robot.joints.map(n => n.matrixWorld.toArray()), matrices);
});
check('重新摆放基座后，实例 FK 能正确更新世界坐标', () => {
  placement.position.add(new THREE.Vector3(2, -1, 3));
  placement.rotateY(0.4);
  placement.updateMatrixWorld(true);
  const movedFK = new Kinematics(robot.getKinematicConfig());
  assert(movedFK.verifyFK(robot.getJointAngles(), robot.getTCPPose().matrixWorld).isPassed);
  robot.root.position.x += 2;
  robot.root.rotateY(0.2);
  const directMoveFK = new Kinematics(robot.getKinematicConfig());
  assert(directMoveFK.verifyFK(robot.getJointAngles(), robot.getTCPPose().matrixWorld).isPassed);
});
check('缺关节、重复关节与错误层级立即报错', () => {
  assert.throws(() => new RobotModel(new THREE.Group()), /唯一/);
  const malformed = scene.clone(true);
  malformed.add(malformed.getObjectByName('J2'));
  assert.throws(() => new RobotModel(malformed), /祖先链/);
  const duplicate = scene.clone(true);
  const node = new THREE.Group(); node.name = 'J1'; duplicate.add(node);
  assert.throws(() => new RobotModel(duplicate), /唯一/);
});
const output = { ...suite, additionalChecks: checks, source: ROBOT_CONFIG.source, sha256: ROBOT_CONFIG.sha256, unitNote: ROBOT_CONFIG.unitNote };
if (process.argv[2]) {
  const dir = resolve(process.argv[2]);
  await mkdir(dir, { recursive: true });
  await writeFile(resolve(dir, 'FK验证原始数据.json'), JSON.stringify(output, null, 2));
  const header = 'case,position_error_m,matrix_max_diff,passed,j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,x_m,y_m,z_m';
  await writeFile(resolve(dir, 'FK验证113组.csv'), header + '\n' + suite.rows.map(r => [r.name, r.positionError, r.matrixMaxDiff, r.isPassed, ...r.anglesRad, ...r.position].join(',')).join('\n') + '\n');
}
console.log(JSON.stringify({ cases: suite.total, passed: suite.passed, maxPositionError: suite.maxPositionError, maxMatrixDiff: suite.maxMatrixDiff, additionalChecks: checks }, null, 2));
