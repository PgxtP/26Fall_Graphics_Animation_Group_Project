import fs from 'node:fs';
import assert from 'node:assert/strict';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const urls = new Map();
function moduleUrl(file) {
  if (urls.has(file)) return urls.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8').replace(/from '([^']+)'/g, (match, name) => {
    const target = name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name));
    return `from '${moduleUrl(target)}'`;
  });
  const url = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  urls.set(file, url); return url;
}
const THREE = await import(moduleUrl('js/vendor/three/build/three.core.js'));
const { threePointFrame } = await import(moduleUrl('js/calibration-math.js'));
const frame = threePointFrame([1, 2, 3], [3, 2, 3], [1, 3, 4]);
assert(Math.abs(frame.determinant - 1) < 1e-12); assert(frame.roundTripError < 1e-12); assert(frame.axisError < 1e-12);
const independent = new THREE.Vector3(9, -3, 7);
assert(independent.clone().applyMatrix4(new THREE.Matrix4().fromArray(frame.inverse)).applyMatrix4(new THREE.Matrix4().fromArray(frame.matrix)).distanceTo(independent) < 1e-12);
assert.throws(() => threePointFrame([0, 0, 0], [1, 0, 0], [2, 0, 0]), /共线/);
assert.throws(() => threePointFrame([0, 0, 0], [0, 0, 0], [2, 0, 0]), /重合/);
const { createMotion } = await import(moduleUrl('js/pendant-motion.js'));
const node = new THREE.Object3D();
const joint = { name: 'J1', type: 'revolute', node, axis: new THREE.Vector3(0, 1, 0), restQuaternion: node.quaternion.clone(), lower: -1, upper: 1, angle: 0 };
const motion = createMotion({ joints: [joint], robot: node }, { joints: [], root: new THREE.Object3D() });
assert.throws(() => motion.target('J1', .5), /使能/);
motion.state.enabled = true; motion.state.speed = 100;
assert.equal(motion.target('J1', 99), 1);
for (let i = 0; i < 20; i++) motion.update(.05);
assert(Math.abs(joint.angle - Math.PI / 6) < 1e-10); // 30 degrees/second.
motion.state.estop = true; const stoppedAngle = joint.angle; motion.update(.05); assert.equal(joint.angle, stoppedAngle); assert.equal(motion.busy(), false);
motion.state.estop = false; motion.state.jog = { name: 'J1', direction: -1 }; motion.update(.05); assert(joint.angle < stoppedAngle);
motion.stop(); const afterRelease = joint.angle; motion.update(.05); assert.equal(joint.angle, afterRelease);
console.log('PASS: three-point frames/roundtrip, speed, enable, emergency stop, jog release and clamping.');
