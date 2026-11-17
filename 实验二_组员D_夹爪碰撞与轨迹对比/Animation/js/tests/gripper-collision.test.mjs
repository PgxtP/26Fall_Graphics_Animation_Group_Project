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
const { createMotion, checkGripperCollision, resolveGripperScene } = await import(url('js/pendant-motion.js'));

const box = (w, h, d, name, position = [0, 0, 0]) => {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d));
  mesh.name = name;
  mesh.position.fromArray(position);
  return mesh;
};

/** 与 Workstation 同构的几何夹爪：掌部 + 左右指，挂在法兰下（blockSize=0.08）。 */
function buildGripper() {
  const blockSize = 0.08;
  const flange = new THREE.Object3D();
  flange.name = 'link_flange';
  const gripper = new THREE.Group();
  gripper.name = 'tcp-gripper';
  gripper.add(box(blockSize * 1.8, blockSize * 0.22, blockSize * 0.6, 'gripper-palm', [0, blockSize * 0.35, 0]));
  gripper.add(box(blockSize * 0.14, blockSize * 0.85, blockSize * 0.45, 'gripper-left', [-blockSize * 0.7, -blockSize * 0.15, 0]));
  gripper.add(box(blockSize * 0.14, blockSize * 0.85, blockSize * 0.45, 'gripper-right', [blockSize * 0.7, -blockSize * 0.15, 0]));
  flange.add(gripper);
  return { flange, gripper };
}

/** 场景：基座 + 工位台面 + 可绕 Y 旋转的 J1 臂，夹爪装在臂端半径 1.2 处。 */
function buildScene({ tableCenter = [2.0, 0.5, 0], tableSize = 1 } = {}) {
  const robot = new THREE.Object3D();
  robot.add(box(0.4, 0.4, 0.4, 'link_base', [0, 0.2, 0]));
  const j1 = new THREE.Object3D();
  j1.name = 'J1';
  robot.add(j1);
  const { flange, gripper } = buildGripper();
  flange.position.set(1.2, 0.5, 0);
  j1.add(flange);

  const table = new THREE.Group();
  table.name = 'table';
  table.add(box(tableSize, tableSize, tableSize, 'table1', tableCenter));
  const scene = new THREE.Scene();
  scene.add(robot);
  scene.add(table);

  const joint = {
    name: 'J1', node: j1, type: 'revolute', axis: new THREE.Vector3(0, 1, 0),
    angle: 0, restQuaternion: j1.quaternion.clone(), lower: -1, upper: 1,
  };
  const api = {
    robot, joints: [joint], tcp: flange, floorY: 0,
    workstation: { table, gripper, workpiece: box(0.08, 0.08, 0.08, 'workpiece', [1.2, 1.05, 0]), attached: false, scene },
  };
  return { scene, robot, table, api, joint, flange, gripper };
}

// 1. 远离障碍物：不判定碰撞，且给出最近间距
{
  const { api, flange, gripper, table } = buildScene();
  api.robot.updateMatrixWorld(true);
  const result = checkGripperCollision(gripper, [{ object: table, name: '工作台' }], { probeDistance: 0.5 });
  assert.equal(result.colliding, false, '悬空夹爪不应判定碰撞');
  assert.equal(result.gripperTriangles, 36, '三个盒体应展开为 36 个三角面片');
  assert.ok(Math.abs(result.minDistance - 0.228) < 0.005, `最近间距应约 0.228，实际 ${result.minDistance}`);
  assert.equal(result.hits.length, 0);
  // 显式传入法兰（包含夹爪子树）同样可用
  assert.equal(checkGripperCollision(flange, [table]).colliding, false);
}

// 2. 穿透：夹爪推进台面内部
{
  const { api, gripper, table, flange } = buildScene();
  flange.position.set(1.55, 0.5, 0); // 掌部 x∈[1.478, 1.622] 嵌入台面 x∈[1.5, 2.5]
  api.robot.updateMatrixWorld(true);
  const result = checkGripperCollision(gripper, [{ object: table, name: '工作台' }]);
  assert.equal(result.colliding, true, '嵌入台面必须判定穿透');
  assert.equal(result.hits[0].kind, 'penetration');
  assert.equal(result.hits[0].obstacle, '工作台');
  assert.match(result.reason, /穿透/);
  assert.equal(result.minDistance, 0);
}

// 3. 安全间距：margin 控制「贴近但未穿透」（真实间距 0.008）
{
  const { api, gripper } = buildScene();
  const wall = box(0.2, 2, 2, '挡板', [1.38, 0, 0]); // 左面 x=1.28，夹爪右缘 x=1.272
  api.robot.updateMatrixWorld(true);
  const loose = checkGripperCollision(gripper, [{ object: wall, name: '挡板' }], { margin: 0 });
  const tight = checkGripperCollision(gripper, [{ object: wall, name: '挡板' }], { margin: 0.01 });
  assert.equal(loose.colliding, false, 'margin=0 时 0.008 的间距不应拦截');
  assert.ok(Math.abs(loose.minDistance - 0.008) < 0.004, `minDistance 应接近真实间距 0.008，实际 ${loose.minDistance}`);
  assert.equal(tight.colliding, true, 'margin=0.01 时应以 clearance 拦截');
  assert.equal(tight.hits[0].kind, 'clearance');
  assert.ok(tight.hits[0].distance <= 0.01);
  assert.match(tight.reason, /安全间距/);
}

// 4. 已挂载工件参与检测：工件伸出夹爪之外，先于夹爪本体撞到台面
{
  const { api, gripper, table, flange } = buildScene();
  const workpiece = box(0.3, 0.3, 0.3, 'workpiece', [0.4, -0.4, 0]);
  flange.add(workpiece);
  api.workstation.attached = true;
  api.workstation.workpiece = workpiece;
  api.robot.updateMatrixWorld(true);
  const parts = resolveGripperScene(api).parts;
  assert.deepEqual(parts.map((part) => part.name), ['tcp-gripper', 'workpiece'], '夹爪 + 已挂载工件应一起检测');
  const gripperOnly = checkGripperCollision([gripper], [{ object: table, name: '工作台' }]);
  const withPart = checkGripperCollision(parts, [{ object: table, name: '工作台' }]);
  assert.equal(gripperOnly.colliding, false, '夹爪本体尚未碰到台面');
  assert.equal(withPart.colliding, true, '工件嵌入台面应被拦截');
  assert.equal(resolveGripperScene(api).obstacles.some((entry) => entry.name === '工件'), false, '已挂载工件不应再作为障碍物');
}

// 5. 自身跳过：把夹爪或它的祖先当障碍物时不得判为碰撞
{
  const { api, gripper, flange, table } = buildScene();
  api.robot.updateMatrixWorld(true);
  const self = checkGripperCollision([gripper], [gripper], {});
  assert.equal(self.obstaclesChecked, 0, '障碍物就是夹爪本身时应跳过');
  assert.equal(self.colliding, false);
  assert.equal(checkGripperCollision([gripper], [flange], {}).obstaclesChecked, 0, '障碍物是夹爪祖先时应跳过');
  assert.equal(checkGripperCollision([gripper], [table], {}).obstaclesChecked, 1);
}

// 6. 退化输入：没有网格 / 没有障碍物时给出原因而不是抛错
{
  const empty = new THREE.Object3D();
  const result = checkGripperCollision([empty], [], {});
  assert.equal(result.colliding, false);
  assert.match(result.reason, /三角网格/);
  const { api, gripper } = buildScene();
  api.robot.updateMatrixWorld(true);
  const none = checkGripperCollision([gripper], [], {});
  assert.equal(none.colliding, false);
  assert.equal(none.obstaclesChecked, 0);
}

// 7. 刚体旋转不变性：整体绕 Y 轴转 90°，碰撞结论与间距都不变
{
  const one = buildScene();
  one.api.robot.updateMatrixWorld(true);
  const before = checkGripperCollision(one.gripper, [{ object: one.table, name: '工作台' }], { probeDistance: 0.5 });
  const two = buildScene();
  const rig = new THREE.Group();
  rig.rotation.y = Math.PI / 2;
  rig.add(two.api.robot, two.table);
  rig.updateMatrixWorld(true);
  const after = checkGripperCollision(two.gripper, [{ object: two.table, name: '工作台' }], { probeDistance: 0.5 });
  assert.equal(after.colliding, before.colliding, '刚体旋转不应改变碰撞结论');
  assert.ok(Math.abs(after.minDistance - before.minDistance) < 1e-6, `刚体旋转不应改变最近间距：${before.minDistance} vs ${after.minDistance}`);
}

// 8. 运动层集成：J1 推进时被夹爪穿透拦截，位移回滚并留下原因
{
  const { api, joint, gripper, table } = buildScene({ tableCenter: [0.85, 0.5, -0.85] });
  api.robot.updateMatrixWorld(true);
  const motion = createMotion(api, { joints: [], root: new THREE.Object3D() });
  motion.state.enabled = true;
  motion.state.speed = 100;
  assert.equal(motion.getGripperCollisionConfig().enabled, true);
  assert.equal(checkGripperCollision(gripper, [{ object: table, name: '工作台' }]).colliding, false, '初始位形应无碰撞');

  motion.target('J1', 0.55); // 沿圆弧扫向台面
  for (let frame = 0; frame < 400 && !motion.state.collision; frame += 1) motion.update(0.05);
  assert.equal(motion.state.collision, true, '运动过程中应触发夹爪碰撞');
  assert.match(motion.state.collisionReason, /夹爪/);
  assert.equal(motion.state.collisionCount, 1);
  assert.equal(motion.state.targets.size, 0, '碰撞后目标应被清空');
  assert.ok(Math.abs(joint.angle) < 0.55, '碰撞帧必须回滚到未碰撞的角度');
  assert.equal(checkGripperCollision(gripper, [{ object: table, name: '工作台' }]).colliding, false, '回滚后不应仍处于碰撞状态');
  assert.equal(motion.checkGripperCollision().colliding, false, '主动检测与运动层口径一致');
}

// 9. 小幅点动：接近但未穿透时不误报
{
  const { api, joint } = buildScene({ tableCenter: [0.85, 0.5, -0.85] });
  api.robot.updateMatrixWorld(true);
  const motion = createMotion(api, { joints: [], root: new THREE.Object3D() });
  motion.state.enabled = true;
  motion.target('J1', 0.05);
  for (let frame = 0; frame < 200 && motion.state.targets.size; frame += 1) motion.update(0.05);
  assert.equal(motion.state.collision, false, '小幅点动不应误报');
  assert.ok(Math.abs(joint.angle - 0.05) < 1e-6, '目标应正常到位');
}

// 10. 未配置工位模块时退回 flange / tcp 子树，仍不报错
{
  const robot = new THREE.Object3D();
  robot.add(box(0.4, 0.4, 0.4, 'link_base', [0, 0.2, 0]));
  const flange = new THREE.Object3D();
  flange.name = 'link_flange';
  flange.position.set(5, 5, 5);
  flange.add(box(0.1, 0.1, 0.1, 'flange-body'));
  robot.add(flange);
  const api = { robot, joints: [], tcp: flange, floorY: 0 };
  robot.updateMatrixWorld(true);
  const scene = resolveGripperScene(api);
  assert.deepEqual(scene.parts.map((part) => part.name), ['link_flange'], '没有工位模块时退回 flange');
  const result = checkGripperCollision(scene.parts, scene.obstacles);
  assert.equal(result.colliding, false);
  assert.equal(result.obstaclesChecked, 1, '只剩基座一个障碍物');
}

console.log('PASS: gripper-collision.test.mjs: 三角面片穿透、安全间距、已挂载工件联动、自身跳过、刚体旋转不变性、运动层回滚与原因记录、小幅点动不误报、无工位退化。');
