import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..'), cache = new Map();
function url(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8').replace(/from '([^']+)'/g, (_, name) => `from '${url(name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name)))}'`);
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`; cache.set(file, value); return value;
}
const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { createMotion } = await import(url('js/pendant-motion.js'));
const { compileProgram, createProgramRunner } = await import(url('js/movj-program.js'));
const PRESET_POSES_DEG = [
  [0, 0, 0, 0, -180],
  [0, 0, 0, 0, -90],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 90],
  [0, 0, 0, 0, 180],
];
const robot = new THREE.Object3D();
const joints = [120, 120, 90, 120, 180].map((limit, i) => {
  const node = new THREE.Object3D(); robot.add(node);
  return { name: `J${i + 1}`, node, type: 'revolute', axis: new THREE.Vector3(0, 1, 0), angle: 0, restQuaternion: node.quaternion.clone(), lower: -limit * Math.PI / 180, upper: limit * Math.PI / 180 };
});
const api = { robot, joints }, motion = createMotion(api, { joints: [], root: new THREE.Object3D() });
const runner = createProgramRunner(api, motion);
const poses = PRESET_POSES_DEG.map((angles, i) => ({ name: `P${i + 1}`, angles: angles.map((angle) => angle * Math.PI / 180) }));
assert.deepEqual(PRESET_POSES_DEG.map((angles) => angles.slice(0, 4)), Array.from({ length: 5 }, () => [0, 0, 0, 0]));
assert.deepEqual(PRESET_POSES_DEG.map((angles) => angles[4]), [-180, -90, 0, 90, 180]);
const code = poses.map((pose) => `MOVJ ${pose.name}`).join('\n');
assert.equal(compileProgram(code, poses, joints).length, 5);
assert.throws(() => compileProgram('MOVJ P99', poses, joints), /未记录/);
assert.throws(() => compileProgram('MOVL P1', poses, joints), /格式/);
assert.throws(() => compileProgram('MOVJ P1\nMOVJ P2', poses, joints), /至少4/);
const invalid = structuredClone(poses); invalid[0].angles[2] = Math.PI;
assert.throws(() => compileProgram(code, invalid, joints), /超限/);
assert.throws(() => runner.start(code, poses), /允许/);
motion.state.enabled = true; motion.state.speed = 100;
runner.start(code, poses);
assert.throws(() => motion.target('J1', 0), /程序正在执行/);
const reached = [];
let scannedRadians = 0;
for (let frame = 0; frame < 2000 && runner.state.running; frame++) {
  const before = joints.map((j) => j.angle), index = runner.state.index;
  motion.update(.05);
  if (index > 0) {
    const delta = joints[4].angle - before[4];
    assert(delta >= -1e-10, '扫描阶段应保持正向旋转');
    scannedRadians += delta;
  }
  joints.forEach((j, i) => assert(Math.abs(j.angle - before[i]) <= Math.PI / 6 * .05 + 1e-8));
  if (!motion.busy()) reached.push({ index, angle: joints[4].angle });
  runner.update();
}
assert.equal(runner.state.running, false); assert.match(runner.state.message, /执行完成/);
assert.equal(reached.length, 5); reached.forEach((point, i) => assert(Math.abs(point.angle - poses[i].angles[4]) < 1e-9));
assert(Math.abs(scannedRadians - 2 * Math.PI) < 1e-8, 'P1 到 P5 应累计旋转 360°');
assert.equal(motion.state.programActive, false);
runner.start(code, poses); motion.update(.05); motion.stop(); runner.update();
assert.equal(runner.state.running, false); assert.match(runner.state.message, /中断/);
runner.start(code, poses); motion.state.estop = true; const angle = joints[4].angle; motion.update(.05); runner.update();
assert.equal(joints[4].angle, angle); assert.equal(runner.state.running, false);
motion.state.estop = false; motion.state.enabled = true;
motion.moveJ([.5, .25, 0, 0, joints[4].angle]);
for (let i = 0; i < 5; i++) motion.update(.05);
assert(Math.abs(joints[0].angle / .5 - joints[1].angle / .25) < 1e-10);
// A collision during a program must interrupt it rather than advance to the next point.
motion.stop();
const base = new THREE.Mesh(new THREE.BoxGeometry(2, 2, 2));
base.name = 'link_base';
robot.add(base);
const penetratingLink = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
penetratingLink.name = 'link_1';
penetratingLink.position.y = -1;
robot.add(penetratingLink);
api.floorY = 0;
runner.start(code, poses);
motion.update(.05);
runner.update();
assert.equal(motion.state.collision, true);
assert.equal(runner.state.running, false, '碰撞后程序必须中断');
assert.equal(runner.state.index, 0, '碰撞后不能跳到下一个位姿');
assert.match(runner.state.message, /中断/);
console.log('PASS: MOVJ runner with rotation fixture, limits, synchronized interpolation, speed bound, stop, emergency and collision interruption.');

