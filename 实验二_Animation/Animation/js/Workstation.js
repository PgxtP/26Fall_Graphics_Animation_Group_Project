import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// Preserve the complete world matrix, including a possible shear from scaled
// ancestors. Object3D.attach alone only supports decomposable TRS transforms.
function reparentKeepingWorld(parent, wp) {
  if (!parent?.isObject3D || !wp?.isObject3D) throw new Error('挂载需要两个 THREE.Object3D');
  for (let node = parent; node; node = node.parent) {
    if (node === wp) throw new Error('不能将工件挂载到自身或其子节点');
  }
  parent.updateWorldMatrix(true, false);
  wp.updateWorldMatrix(true, false);
  const determinant = parent.matrixWorld.determinant();
  if (!parent.matrixWorld.elements.every(Number.isFinite) || !Number.isFinite(determinant) || determinant === 0 ||
      !wp.matrixWorld.elements.every(Number.isFinite) || wp.matrixWorld.determinant() === 0) throw new Error('父节点或工件的世界矩阵无效');
  if (wp.parent === parent) return wp;
  const local = parent.matrixWorld.clone().invert().multiply(wp.matrixWorld);
  const autoUpdate = wp.matrixAutoUpdate;
  local.decompose(wp.position, wp.quaternion, wp.scale);
  const composed = new THREE.Matrix4().compose(wp.position, wp.quaternion, wp.scale);
  const scale = Math.max(1, ...local.elements.map(Math.abs));
  const hasShear = local.elements.some((value, i) => Math.abs(value - composed.elements[i]) > scale * 1e-10);
  parent.add(wp);
  wp.matrix.copy(local);
  wp.matrixAutoUpdate = autoUpdate && !hasShear;
  wp.matrixWorldNeedsUpdate = true;
  wp.updateWorldMatrix(false, true);
  return wp;
}

/** 将工件改挂到法兰，保持抓取瞬间的世界位姿。底层接口不进行距离/碰撞判定。 */
export function attachWorkpiece(flange, wp) {
  return reparentKeepingWorld(flange, wp);
}

/** 将工件脱开到场景，保持释放瞬间的世界位姿，不自动放回工作台。 */
export function detachWorkpiece(scene, wp) {
  if (!scene?.isScene) throw new Error('脱开需要 THREE.Scene');
  return reparentKeepingWorld(scene, wp);
}

/** 工位与几何夹爪；距离判断仅用于抓取联动，不代替碰撞检测或物理仿真。 */
export class Workstation {
  constructor(scene, flange, options = {}) {
    if (!scene?.isScene || !flange?.isObject3D) throw new Error('工位需要 THREE.Scene 与末端 flange');
    if (options.graspTolerance !== undefined && (!Number.isFinite(options.graspTolerance) || options.graspTolerance <= 0)) throw new Error('抓取容差必须大于0');
    this.scene = scene; this.flange = flange; this.options = options;
    this.table = new THREE.Group(); this.table.name = 'table';
    this.workpiece = null;
    this.gripper = new THREE.Group(); this.gripper.name = 'tcp-gripper';
    this.gripper.matrixAutoUpdate = false;
    this.gripperState = 'OPEN';
    this.fingers = [];
  }

  async initialize() {
    const span = this.options.modelSpan ?? 12;
    if (!Number.isFinite(span) || span <= 0) throw new Error('工位尺度必须大于0');
    const model = (await new GLTFLoader().loadAsync('./images/table1.glb')).scene;
    model.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(model), size = bounds.getSize(new THREE.Vector3()), center = bounds.getCenter(new THREE.Vector3());
    if (bounds.isEmpty() || Math.max(size.x, size.z) <= 0) throw new Error('工作台模型没有有效尺寸');
    const scale = span * .45 / Math.max(size.x, size.z);
    const tableDepth = size.z * scale;
    model.scale.multiplyScalar(scale);
    model.position.add(new THREE.Vector3(-center.x * scale, -bounds.min.y * scale, -center.z * scale));
    model.traverse((node) => { if (node.isMesh) { node.castShadow = true; node.receiveShadow = true; } });
    this.table.add(model);
    const robotCenter = this.options.robotCenter ?? new THREE.Vector3();
    this.tableHeight = size.y * scale;
    this.blockSize = span * .065;
    // 先让整张桌子避开机械臂当前包围盒，而不是把桌心推向可达点。
    let robotRoot = this.flange;
    while (robotRoot.parent && robotRoot.parent !== this.scene) robotRoot = robotRoot.parent;
    robotRoot.updateWorldMatrix(true, true);
    const robotBounds = new THREE.Box3().setFromObject(robotRoot);
    if (robotBounds.isEmpty()) throw new Error('机械臂没有有效包围盒，无法摆放工作台');
    const clearance = span * .06;
    const edgeMargin = span * .015;
    // 工作台与基座中线对齐，置于机械臂工作的 −Z 一侧。
    const blockX = 0;
    const blockZ = tableDepth / 2 - this.blockSize / 2 - edgeMargin;
    const base = robotRoot.getObjectByName('link_base');
    const baseCenter = base ? base.getWorldPosition(new THREE.Vector3()) : robotCenter;
    this.table.position.set(baseCenter.x, this.options.floorY ?? 0,
      robotBounds.min.z - clearance - tableDepth / 2);
    this.scene.add(this.table);
    this.workpiece = new THREE.Mesh(new THREE.BoxGeometry(this.blockSize, this.blockSize, this.blockSize), new THREE.MeshStandardMaterial({ color: 0xe99b45, roughness: .55 }));
    this.workpiece.name = 'workpiece'; this.workpiece.castShadow = true; this.workpiece.receiveShadow = true;
    this.table.add(this.workpiece);
    this.placementMatrix = new THREE.Matrix4().makeTranslation(blockX, this.tableHeight + this.blockSize / 2, blockZ);
    this.markers = {};
    const h = this.blockSize / 2, top = h + this.blockSize * .01;
    const locations = { O: [-h, top, h], Px: [h, top, h], Pxy: [-h, top, -h] };
    const colors = { O: 0xffe06b, Px: 0xff7070, Pxy: 0x73baff };
    for (const [name, position] of Object.entries(locations)) {
      const point = new THREE.Mesh(new THREE.SphereGeometry(this.blockSize * .045, 12, 8), new THREE.MeshBasicMaterial({ color: colors[name] }));
      point.name = `mark-${name}`; point.position.fromArray(position); this.workpiece.add(point); this.markers[name] = point;
      const canvas = document.createElement('canvas'); canvas.width = 128; canvas.height = 64;
      const ctx = canvas.getContext('2d'); ctx.font = 'bold 38px sans-serif'; ctx.fillStyle = '#ffffff'; ctx.textAlign = 'center'; ctx.fillText(name, 64, 45);
      const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
      label.scale.set(this.blockSize * .45, this.blockSize * .225, 1); label.position.set(0, this.blockSize * .18, 0); point.add(label);
    }
    this.createGripper();
    this.resetWorkpiece();
    return this;
  }

  get attached() { return !!this.workpiece && this.workpiece.parent === this.flange; }

  createGripper() {
    if (this.gripper.parent) return;
    const s = this.blockSize;
    const metal = new THREE.MeshStandardMaterial({ color: 0x8294a7, metalness: .65, roughness: .35 });
    const palm = new THREE.Mesh(new THREE.BoxGeometry(s * 1.8, s * .22, s * .6), metal);
    palm.name = 'gripper-palm'; palm.position.y = s * .35;
    this.gripper.add(palm);
    for (const side of [-1, 1]) {
      const finger = new THREE.Mesh(new THREE.BoxGeometry(s * .14, s * .85, s * .45), metal);
      finger.name = side < 0 ? 'gripper-left' : 'gripper-right';
      finger.userData.side = side; finger.position.y = -s * .15;
      this.gripper.add(finger); this.fingers.push(finger);
    }
    this.gripper.traverse((node) => { if (node.isMesh) node.castShadow = true; });
    this.flange.add(this.gripper);
    this.updateGripper();
  }

  /** TCP 改变时/每帧调用，使夹爪根节点跟随当前工具偏移。 */
  updateGripper() {
    const offset = this.options.getTCPOffsetMatrix?.() ?? new THREE.Matrix4();
    if (!offset?.isMatrix4 || !offset.elements.every(Number.isFinite) || offset.determinant() === 0) throw new Error('TCP 偏移矩阵无效');
    this.gripper.matrix.copy(offset); this.gripper.matrixWorldNeedsUpdate = true;
    for (const finger of this.fingers) {
      finger.position.x = finger.userData.side * this.blockSize * (this.gripperState === 'OPEN' ? .82 : .57);
    }
    this.gripper.userData.gripState = this.gripperState;
    this.gripper.updateWorldMatrix(true, true);
  }

  get gripStatus() {
    if (!this.workpiece) return { state: this.gripperState, attached: false, canGrab: false, distance: null, tolerance: null };
    this.updateGripper();
    const tcp = this.gripper.getWorldPosition(new THREE.Vector3());
    this.workpiece.updateWorldMatrix(true, false);
    const contact = new THREE.Vector3(0, this.blockSize / 2, 0).applyMatrix4(this.workpiece.matrixWorld);
    const distance = tcp.distanceTo(contact);
    const tolerance = this.options.graspTolerance ?? this.blockSize * .2;
    if (!Number.isFinite(tolerance) || tolerance <= 0) throw new Error('抓取容差必须大于0');
    return { state: this.gripperState, attached: this.attached, distance, tolerance, canGrab: distance <= tolerance };
  }

  canGrab() { return this.gripStatus.canGrab; }

  /** 供 GRIP 指令调用；CLOSE 距离不足时只闭爪，不吸附远处工件。 */
  setGrip(state) {
    if (!this.workpiece) throw new Error('工位尚未初始化');
    if (!['OPEN', 'CLOSE'].includes(state)) throw new Error('夹爪指令必须是 OPEN 或 CLOSE');
    if (state === 'CLOSE' && !this.attached && this.canGrab()) attachWorkpiece(this.flange, this.workpiece);
    if (state === 'OPEN' && this.attached) detachWorkpiece(this.scene, this.workpiece);
    this.gripperState = state;
    this.updateGripper();
    const status = this.gripStatus;
    this.options.onGripChange?.(status);
    return status;
  }

  attachWorkpiece() {
    if (!this.workpiece) throw new Error('工位尚未初始化');
    if (!this.attached && !this.canGrab()) throw new Error('TCP 未到工件顶面抓取点，请先移动机械臂接近工件');
    this.setGrip('CLOSE');
    return this.workpiece;
  }

  detachWorkpiece() {
    this.setGrip('OPEN');
    return this.workpiece;
  }

  /** 明确的场景重置操作；与释放到当前位置的 OPEN 分开。 */
  resetWorkpiece() {
    if (!this.workpiece) throw new Error('工位尚未初始化');
    this.table.add(this.workpiece);
    this.workpiece.matrixAutoUpdate = true;
    this.placementMatrix.decompose(this.workpiece.position, this.workpiece.quaternion, this.workpiece.scale);
    // The table subtree is used by the existing motion guard as one static
    // obstacle. A grasp target inside that subtree would enlarge its AABB and
    // stop the open jaws before they can reach the top-center contact point.
    // Resolve the table-relative placement, then keep the independent workpiece
    // at that world pose under the scene, just like an ordinarily released part.
    detachWorkpiece(this.scene, this.workpiece);
    this.gripperState = 'OPEN';
    this.updateGripper();
    this.scene.updateMatrixWorld(true);
    this.options.onGripChange?.(this.gripStatus);
    return this.workpiece;
  }

  getCalibrationPoints() {
    this.scene.updateMatrixWorld(true);
    return Object.fromEntries(Object.entries(this.markers).map(([name, node]) => [name, node.getWorldPosition(new THREE.Vector3())]));
  }
}
