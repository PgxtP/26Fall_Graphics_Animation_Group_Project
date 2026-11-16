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

// 1. Mock structure with link_base, J3, J4, and link_3
const robot = new THREE.Object3D();
const baseMesh = new THREE.Mesh(new THREE.BoxGeometry(2, 4, 2));
baseMesh.name = 'link_base';
baseMesh.position.set(0, 2, 0);
robot.add(baseMesh);

const j3 = new THREE.Object3D();
j3.name = 'J3';
const j4 = new THREE.Object3D();
j4.name = 'J4';
robot.add(j3);
robot.add(j4);

const link3Mesh = new THREE.Mesh(new THREE.BoxGeometry(0.5, 4, 0.5));
link3Mesh.name = 'link_3';
robot.add(link3Mesh);

const link1Mesh = new THREE.Mesh(new THREE.BoxGeometry(2, 4, 2));
link1Mesh.name = 'link_1';
link1Mesh.position.set(0, 2, 0);
robot.add(link1Mesh);

const joints = [120, 120, 90, 120, 180].map((limit, i) => {
  const node = new THREE.Object3D(); robot.add(node);
  return { name: `J${i + 1}`, node, type: 'revolute', axis: new THREE.Vector3(0, 1, 0), angle: 0, restQuaternion: node.quaternion.clone(), lower: -limit * Math.PI / 180, upper: limit * Math.PI / 180 };
});
const api = { robot, joints, floorY: 0 };
const motion = createMotion(api, { joints: [], root: new THREE.Object3D() });
motion.state.enabled = true;

// Test 1: When J3 and J4 are safe (e.g. Y > 6, away from base)
j3.position.set(0, 8, 3);
j4.position.set(0, 6, 5);
link3Mesh.position.set(0, 7, 4);
robot.updateMatrixWorld(true);
assert.equal(motion.state.collision, false);

// Test 2: When J3-J4 segment intersects the base column (base is [-1, 1] x [0, 4] x [-1, 1])
j3.position.set(0, 3, 2);
j4.position.set(0, 3, -2); // The segment passes through (0, 3, 0) inside base!
link3Mesh.position.set(0, 3, 0);
robot.updateMatrixWorld(true);

// Attempt motion update to trigger collision
motion.target('J1', 0.1);
motion.update(0.05);
assert.equal(motion.state.collision, true, 'Should detect J3-J4 segment penetrating base!');
assert.equal(motion.state.targets.size, 0, 'Target should be cleared upon collision');

// Test 3: Floor collision check
motion.state.collision = false;
j3.position.set(0, 8, 3);
j4.position.set(0, 6, 5);
link3Mesh.position.set(0, -0.1, 4); // Dips below floorY=0
robot.updateMatrixWorld(true);
motion.target('J2', 0.1);
motion.update(0.05);
assert.equal(motion.state.collision, true, 'Should detect link touching floor!');

// Test 4: Link 3 vs Link 1 shoulder penetration check
motion.state.collision = false;
link1Mesh.position.set(0, 0, 0);
link3Mesh.position.set(0.5, 0.5, 0.5); // inside localShoulderBox
j3.position.set(0, 8, 3);
j4.position.set(0, 6, 5);
robot.updateMatrixWorld(true);
motion.target('J1', 0.1);
motion.update(0.05);
assert.equal(motion.state.collision, true, 'Should detect link_3 penetrating link_1 shoulder!');


// A broad link face can cut through the root even when no vertex or
// J3–J4 centerline sample lies inside its box.
{
  const root = new THREE.Object3D();
  const base = new THREE.Mesh(new THREE.BoxGeometry(2, 4, 2));
  base.name = 'link_base';
  base.position.y = 2;
  root.add(base);
  const face = new THREE.BufferGeometry();
  face.setAttribute('position', new THREE.Float32BufferAttribute([
    -2, 2, 0,  2, 2, 0,  0, 6, 0,
  ], 3));
  face.setIndex([0, 1, 2]);
  const link = new THREE.Mesh(face);
  link.name = 'link_3';
  root.add(link);
  const axis3 = new THREE.Object3D(), axis4 = new THREE.Object3D();
  axis3.name = 'J3'; axis4.name = 'J4';
  axis3.position.set(5, 3, 0); axis4.position.set(5, 5, 0);
  root.add(axis3, axis4);
  const axis = new THREE.Object3D();
  root.add(axis);
  const joint = { name: 'J1', node: axis, type: 'revolute', axis: new THREE.Vector3(0, 1, 0),
    angle: 0, restQuaternion: axis.quaternion.clone(), lower: -1, upper: 1 };
  const faceMotion = createMotion({ robot: root, joints: [joint], floorY: 0 },
    { joints: [], root: new THREE.Object3D() });
  faceMotion.state.enabled = true;
  faceMotion.target('J1', 0.1);
  faceMotion.update(0.05);
  assert.equal(faceMotion.state.collision, true, 'Crossing mesh face must hit the base');
  assert.equal(joint.angle, 0, 'Colliding movement must roll back');

  link.position.x = 5;
  faceMotion.state.collision = false;
  faceMotion.target('J1', 0.1);
  faceMotion.update(0.05);
  assert.equal(faceMotion.state.collision, false, 'Separated mesh must remain movable');
}
console.log('PASS: collision.test.mjs: J3-J4 segment vs base detection, link_3 vs link_1 shoulder self-collision, floor penetration check, automatic target rollback.');
