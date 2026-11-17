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
const { calibrateUserFrameThreePoints, worldToUserFrame, createTCPOffset } = await import(url('js/Calibration.js'));
const O = [1, 2, 3], Px = [1, 4, 3], Pxy = [-1, 3, 3];
const userMatrix = calibrateUserFrameThreePoints(O, Px, Pxy);
assert(userMatrix.isMatrix4);
const cases = [
  { world: O, expectedUser: [0, 0, 0] },
  { world: Px, expectedUser: [2, 0, 0] },
  { world: Pxy, expectedUser: [1, 2, 0] },
  { world: [-2, 4, 7], expectedUser: [2, 3, 4] },
];
for (const record of cases) {
  const value = worldToUserFrame(record.world, userMatrix);
  record.actualUser = value.toArray();
  record.expectedError = value.distanceTo(new THREE.Vector3().fromArray(record.expectedUser));
  record.roundTripError = value.clone().applyMatrix4(userMatrix.clone().invert()).distanceTo(new THREE.Vector3().fromArray(record.world));
  assert(record.expectedError < 1e-12); assert(record.roundTripError < 1e-12);
}
assert.throws(() => calibrateUserFrameThreePoints([0, 0, 0], [1, 0, 0], [2, 0, 0]), /共线/);
assert.throws(() => calibrateUserFrameThreePoints(O, O, Pxy), /重合/);
assert.throws(() => worldToUserFrame([NaN, 0, 0], userMatrix), /有限/);
assert.throws(() => createTCPOffset([0, 0, 0], [Infinity, 0, 0]), /有限/);
const tcpOffsetMatrix = createTCPOffset([.1, .2, .3], [0, 0, Math.PI / 2]);
assert(tcpOffsetMatrix.isMatrix4);
const transformed = new THREE.Vector3(1, 0, 0).applyMatrix4(tcpOffsetMatrix);
const tcpError = transformed.distanceTo(new THREE.Vector3(.1, 1.2, .3)); assert(tcpError < 1e-12);
const worldTool = new THREE.Matrix4().makeTranslation(5, 0, 0).multiply(tcpOffsetMatrix);
const worldOrigin = new THREE.Vector3().setFromMatrixPosition(worldTool);
assert(worldOrigin.distanceTo(new THREE.Vector3(5.1, .2, .3)) < 1e-12);
const data = { source: 'deterministic-mathematical-verification-not-physical-measurement',
  points: { O, Px, Pxy }, matrixDirection: 'world-to-user', userMatrixColumnMajor: userMatrix.toArray(), cases,
  tcp: { trans: [.1, .2, .3], rpyRad: [0, 0, Math.PI / 2], matrixDirection: 'tool-to-flange', matrixColumnMajor: tcpOffsetMatrix.toArray(),
    testToolPoint: [1, 0, 0], expectedFlangePoint: [.1, 1.2, .3], actualFlangePoint: transformed.toArray(), error: tcpError,
    flangeWorldTranslation: [5, 0, 0], toolWorldOrigin: worldOrigin.toArray() } };
fs.writeFileSync(path.join(root, '组员B_坐标验证数据.json'), JSON.stringify(data, null, 2) + '\n');
console.log('PASS: named Calibration.js interfaces, explicit world-to-user convention, Gram–Schmidt with skew plane vector, independent expected coordinates, TCP rotation/translation, degenerate inputs.');
