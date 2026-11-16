import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

// Resolve the same pinned local three.js module used by the Live Server import map.
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const cache = new Map();
function moduleURL(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8').replace(/from '([^']+)'/g, (_, name) => {
    const target = name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name));
    return `from '${moduleURL(target)}'`;
  });
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value);
  return value;
}
const THREE = await import(moduleURL('js/vendor/three/build/three.core.js'));
const { fitSphereCenter, fitTCPFromPoses } = await import(moduleURL('js/Calibration.js'));
const offset = new THREE.Vector3(.7, -.4, 1.2), center = new THREE.Vector3(2, -3, 4);
const angles = [[.2, -.1, .3], [.8, .2, .3], [-.5, 1.1, -.4], [.6, -.9, .8]];
function pose(rotation, scale = 1, noise = new THREE.Vector3()) {
  const quaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(...rotation, 'ZYX'));
  const origin = center.clone().sub(offset.clone().multiplyScalar(scale).applyQuaternion(quaternion)).add(noise);
  return new THREE.Matrix4().compose(origin, quaternion, new THREE.Vector3(scale, scale, scale));
}
const positions = (poses) => poses.map((matrix) => new THREE.Vector3().setFromMatrixPosition(matrix));

test('four nonzero orientations recover an independent world center and local TCP offset', () => {
  const poses = angles.map((rotation) => pose(rotation));
  const originals = poses.map((matrix) => matrix.toArray());
  const result = fitTCPFromPoses(poses);
  assert(result.center.distanceTo(center) < 1e-11);
  assert(result.offset.distanceTo(offset) < 1e-11);
  assert(Math.abs(result.sphere.radius - offset.length()) < 1e-11);
  assert(result.rms < 1e-11 && result.maxResidual < 1e-11);
  assert(result.sphere.rms < 1e-11);
  assert.equal(result.sampleCount, 4);
  assert(Number.isFinite(result.sphere.condition) && result.sphere.condition >= 1);
  assert.deepEqual(poses.map((matrix) => matrix.toArray()), originals);
  // Check the independently specified tip rather than only self-consistency of the fit.
  for (const matrix of poses) assert(result.offset.clone().applyMatrix4(matrix).distanceTo(center) < 1e-11);
});

test('overdetermined noisy samples report nonzero radial and contact residuals', () => {
  const poses = Array.from({ length: 32 }, (_, i) => pose(
    [Math.sin(i * 1.7) * 1.8, Math.cos(i * .8) * 1.3, Math.sin(i * .4) * 2],
    1, new THREE.Vector3(Math.sin(i * 2.1), Math.cos(i * .3), Math.sin(i * 1.2)).multiplyScalar(.001),
  ));
  const result = fitTCPFromPoses(poses);
  assert(result.center.distanceTo(center) < .002);
  assert(result.offset.distanceTo(offset) < .002);
  assert(result.rms > 1e-5 && result.rms < .004);
  assert(result.sphere.rms > 1e-5 && result.sphere.rms < .004);
  assert.equal(result.residuals.length, 32);
  const independentErrors = poses.map((matrix) => result.offset.clone().applyMatrix4(matrix).distanceTo(result.center));
  assert.deepEqual(result.residuals, independentErrors);
  assert.equal(result.maxResidual, Math.max(...independentErrors));
});

test('four exact sphere points do not falsely prove fixed-tool contact consistency', () => {
  const poses = angles.map((rotation) => pose(rotation));
  const arbitraryPoses = positions(poses).map((point) => new THREE.Matrix4().makeTranslation(point.x, point.y, point.z));
  const result = fitTCPFromPoses(arbitraryPoses);
  assert(result.sphere.rms < 1e-11);
  assert(result.rms > .1, 'inconsistent orientations must produce substantial TCP contact error');
});

test('column-major pose arrays and consistent uniform scale preserve local units', () => {
  const scale = 2.5, poses = angles.map((rotation) => pose(rotation, scale));
  const result = fitTCPFromPoses(poses.map((matrix) => matrix.toArray()));
  assert(result.offset.distanceTo(offset) < 1e-11);
  assert(result.center.distanceTo(center) < 1e-11);
  assert(Math.abs(result.sphere.radius - offset.length() * scale) < 1e-11);
  assert.equal(result.scale, scale);
  assert(result.rms < 1e-11);
});

test('centered fitting is stable under large translations and changes of length unit', () => {
  const direction = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1 / Math.sqrt(3), -1 / Math.sqrt(3), -1 / Math.sqrt(3)]];
  const translated = new THREE.Vector3(1e8, -1e8, 4e8);
  const result = fitSphereCenter(direction.map((p) => new THREE.Vector3(...p).multiplyScalar(3).add(translated).toArray()));
  assert(result.center.distanceTo(translated) < 1e-6);
  assert(Math.abs(result.radius - 3) < 1e-6);
  const tiny = fitSphereCenter(direction.map((p) => p.map((v) => v * 1e-8)));
  assert(tiny.center.length() < 1e-20);
  assert(Math.abs(tiny.radius - 1e-8) < 1e-20);
});

test('reject insufficient, duplicate, coplanar, nearly planar and nonfinite points', () => {
  assert.throws(() => fitSphereCenter([[0, 0, 0]]), /至少/);
  assert.throws(() => fitSphereCenter(Array.from({ length: 4 }, () => [1, 2, 3])), /重合/);
  assert.throws(() => fitSphereCenter([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]]), /共线|共面/);
  assert.throws(() => fitSphereCenter([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]), /共面/);
  assert.throws(() => fitSphereCenter([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 1e-10]]), /共面/);
  assert.throws(() => fitSphereCenter([[0, 0, 0], [1, 0, 0], [0, 1, 0], [NaN, 0, 1]]), /有限/);
});

test('reject malformed, projective, singular, nonuniform, reflected and varying-scale poses', () => {
  const originals = angles.map((rotation) => pose(rotation));
  const replaced = (bad) => [bad, ...originals.slice(1)];
  assert.throws(() => fitTCPFromPoses([]), /至少/);
  assert.throws(() => fitTCPFromPoses(replaced([1, 2, 3])), /16项/);
  const nonfinite = originals[0].toArray(); nonfinite[12] = Infinity;
  assert.throws(() => fitTCPFromPoses(replaced(nonfinite)), /有限/);
  const projective = originals[0].toArray(); projective[3] = .1;
  assert.throws(() => fitTCPFromPoses(replaced(projective)), /仿射/);
  assert.throws(() => fitTCPFromPoses(replaced(new THREE.Matrix4().makeScale(0, 0, 0))), /奇异/);
  assert.throws(() => fitTCPFromPoses(replaced(new THREE.Matrix4().makeScale(1, 2, 1))), /非均匀/);
  assert.throws(() => fitTCPFromPoses(replaced(new THREE.Matrix4().makeScale(-1, 1, 1))), /反射/);
  const shear = new THREE.Matrix4().set(1, .3, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  assert.throws(() => fitTCPFromPoses(replaced(shear)), /非均匀|剪切/);
  assert.throws(() => fitTCPFromPoses(replaced(pose(angles[0], 2))), /缩放必须一致/);
});
