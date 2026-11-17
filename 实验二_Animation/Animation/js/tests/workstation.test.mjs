import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const cache = new Map();
function url(file) {
  if (cache.has(file)) return cache.get(file);
  // These scene-graph tests construct their geometry directly; GLB loading is
  // exercised by the browser checks and must not require a DOM in Node.
  const source = fs.readFileSync(path.join(root, file), 'utf8')
    .replace("import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';", 'class GLTFLoader { loadAsync() { throw new Error("GLB loader is outside this unit test"); } }')
    .replace(/from '([^']+)'/g, (_, name) => {
    const target = name === 'three' ? 'js/vendor/three/build/three.core.js'
      : name.startsWith('three/addons/') ? name.replace('three/addons/', 'js/vendor/three/addons/')
        : path.posix.normalize(path.posix.join(path.posix.dirname(file), name));
    return `from '${url(target)}'`;
  });
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value); return value;
}

const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { Workstation, attachWorkpiece, detachWorkpiece } = await import(url('js/Workstation.js'));
const world = (node) => { node.updateWorldMatrix(true, false); return node.matrixWorld.clone(); };
const closeMatrix = (actual, expected, tolerance = 1e-10) => {
  assert(Math.max(...actual.elements.map((value, i) => Math.abs(value - expected.elements[i]))) < tolerance);
};

function fixture(options = {}) {
  const scene = new THREE.Scene();
  const flange = new THREE.Group(); flange.name = 'link_flange'; scene.add(flange);
  const station = new Workstation(scene, flange, options);
  station.blockSize = .6;
  station.workpiece = new THREE.Mesh(new THREE.BoxGeometry(.6, .6, .6));
  station.table.position.set(2, 0, -3); scene.add(station.table);
  station.placementMatrix = new THREE.Matrix4().makeTranslation(0, 1, 0);
  station.createGripper(); station.resetWorkpiece();
  return { scene, flange, wp: station.workpiece, station };
}

test('exported attachment keeps world translation, rotation and scale under transformed parents', () => {
  const { scene, flange, wp, station } = fixture();
  flange.position.set(3, -2, 5); flange.rotation.set(.3, .2, -.7); flange.scale.setScalar(1.5);
  attachWorkpiece(station.table, wp);
  station.table.rotation.set(.1, -.3, .4);
  wp.rotation.set(-.5, .2, .1); wp.scale.set(.7, 1.3, 1.1);
  const before = world(wp);
  assert.equal(attachWorkpiece(flange, wp), wp);
  assert.equal(wp.parent, flange); closeMatrix(world(wp), before);
  const local = wp.matrix.clone();
  flange.position.x += 2; flange.rotation.y += .5;
  closeMatrix(world(wp), world(flange).multiply(local));
  const releasePose = world(wp);
  assert.equal(detachWorkpiece(scene, wp), wp);
  assert.equal(wp.parent, scene); closeMatrix(world(wp), releasePose);
  flange.position.y += 8;
  closeMatrix(world(wp), releasePose);
});

test('nonuniformly scaled ancestors preserve world matrix without TRS shear loss', () => {
  const { scene, flange, wp } = fixture();
  flange.scale.set(2, 1, .6); flange.rotation.set(.5, -.2, .7);
  wp.rotation.set(.3, .4, .6);
  const before = world(wp);
  attachWorkpiece(flange, wp);
  assert.equal(wp.matrixAutoUpdate, false);
  closeMatrix(world(wp), before);
  detachWorkpiece(scene, wp); closeMatrix(world(wp), before);
});

test('repeated attach and detach operations are idempotent', () => {
  const { scene, flange, wp } = fixture();
  const before = world(wp);
  attachWorkpiece(flange, wp); attachWorkpiece(flange, wp);
  closeMatrix(world(wp), before); assert.equal(flange.children.filter((node) => node === wp).length, 1);
  detachWorkpiece(scene, wp); detachWorkpiece(scene, wp);
  closeMatrix(world(wp), before); assert.equal(scene.children.filter((node) => node === wp).length, 1);
});

test('invalid nodes, cyclic hierarchy and singular parents are rejected before reparenting', () => {
  const { scene, flange, wp } = fixture();
  const originalParent = wp.parent;
  assert.throws(() => attachWorkpiece({}, wp), /Object3D/);
  assert.throws(() => attachWorkpiece(wp, wp), /自身/);
  const child = new THREE.Group(); wp.add(child);
  assert.throws(() => attachWorkpiece(child, wp), /子节点/);
  assert.throws(() => detachWorkpiece(flange, wp), /Scene/);
  flange.scale.x = 0;
  assert.throws(() => attachWorkpiece(flange, wp), /世界矩阵/);
  assert.equal(wp.parent, originalParent); assert.equal(scene.isScene, true);
});

test('distant CLOSE closes the fingers without teleporting or attaching the workpiece', () => {
  const { scene, wp, station } = fixture();
  const before = world(wp), openX = Math.abs(station.fingers[0].position.x);
  assert.equal(station.canGrab(), false);
  const status = station.setGrip('CLOSE');
  assert.equal(status.state, 'CLOSE'); assert.equal(status.attached, false); assert.equal(status.canGrab, false);
  assert(Math.abs(station.fingers[0].position.x) < openX);
  assert.equal(wp.parent, scene); closeMatrix(world(wp), before);
  assert.throws(() => station.attachWorkpiece(), /未到/);
});

test('near CLOSE grips in place and OPEN releases at the current position', () => {
  const notifications = [];
  const { scene, flange, wp, station } = fixture({ onGripChange: (status) => notifications.push(status) });
  flange.position.copy(wp.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0)));
  const before = world(wp);
  assert.equal(station.canGrab(), true);
  assert.equal(station.setGrip('CLOSE').attached, true);
  assert.equal(station.gripper.userData.gripState, 'CLOSE'); closeMatrix(world(wp), before);
  flange.position.x += 1.5;
  const transported = world(wp); assert(Math.abs(transported.elements[12] - before.elements[12] - 1.5) < 1e-10);
  assert.equal(station.setGrip('OPEN').attached, false);
  assert.equal(wp.parent, scene); closeMatrix(world(wp), transported);
  flange.position.z -= 4; closeMatrix(world(wp), transported);
  assert.equal(notifications.at(-1).state, 'OPEN'); assert.equal(notifications.at(-1).attached, false);
});

test('legacy class wrappers link grip state and explicit reset returns to the table placement', () => {
  const { scene, flange, wp, station } = fixture();
  const original = world(wp);
  flange.position.copy(wp.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0)));
  assert.equal(station.attachWorkpiece(), wp); assert.equal(station.gripperState, 'CLOSE');
  flange.position.y += 2;
  const release = world(wp);
  assert.equal(station.detachWorkpiece(), wp); assert.equal(station.gripperState, 'OPEN');
  closeMatrix(world(wp), release);
  assert.equal(station.resetWorkpiece(), wp); assert.equal(wp.parent, scene);
  assert.equal(wp.matrixAutoUpdate, true); closeMatrix(world(wp), original);
});

test('reset resolves transformed table placement but keeps the target out of static obstacle bounds', () => {
  const { scene, wp, station } = fixture();
  station.table.rotation.set(.2, -.6, .3); station.table.scale.set(.8, 1.4, 1.1);
  const expected = world(station.table).multiply(station.placementMatrix);
  station.resetWorkpiece();
  assert.equal(wp.parent, scene);
  assert.equal(station.table.getObjectById(wp.id), undefined);
  closeMatrix(world(wp), expected);
  const rest = world(wp);
  station.table.position.x += 5;
  closeMatrix(world(wp), rest);
});

test('gripper and proximity calculation follow an updated TCP offset', () => {
  let offset = new THREE.Matrix4().makeTranslation(.2, .4, -.1);
  const { flange, station, wp } = fixture({ getTCPOffsetMatrix: () => offset });
  const contact = wp.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0));
  flange.position.copy(contact).sub(new THREE.Vector3(.2, .4, -.1));
  assert(station.gripStatus.distance < 1e-10);
  offset = new THREE.Matrix4().makeTranslation(.2, 1.4, -.1);
  station.updateGripper();
  assert(Math.abs(station.gripStatus.distance - 1) < 1e-10); assert.equal(station.canGrab(), false);
  closeMatrix(station.gripper.matrix, offset);
  assert.throws(() => station.setGrip('INVALID'), /OPEN/);
});
