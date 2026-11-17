import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

/** 工位场景及物块挂载关系；不包含机械臂路径规划、接触检测或物理仿真。 */
export class Workstation {
  constructor(scene, flange, options = {}) {
    if (!scene?.isScene || !flange?.isObject3D) throw new Error('工位需要 THREE.Scene 与末端 flange');
    this.scene = scene; this.flange = flange; this.options = options;
    this.table = new THREE.Group(); this.table.name = 'table';
    this.workpiece = null;
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
    this.detachWorkpiece();
    return this;
  }

  get attached() { return !!this.workpiece && this.workpiece.parent === this.flange; }

  attachWorkpiece() {
    if (!this.workpiece) throw new Error('工位尚未初始化');
    if (this.attached) return this.workpiece;
    const offset = this.options.getTCPOffsetMatrix?.() ?? new THREE.Matrix4();
    if (!offset?.isMatrix4 || !offset.elements.every(Number.isFinite)) throw new Error('TCP 偏移矩阵无效');
    // 明确放到当前 TCP：物块顶面中心与工具尖端重合，然后随法兰运动。
    const grasp = offset.clone().multiply(new THREE.Matrix4().makeTranslation(0, -this.blockSize / 2, 0));
    this.flange.add(this.workpiece);
    grasp.decompose(this.workpiece.position, this.workpiece.quaternion, this.workpiece.scale);
    this.scene.updateMatrixWorld(true);
    return this.workpiece;
  }

  detachWorkpiece() {
    if (!this.workpiece) throw new Error('工位尚未初始化');
    this.table.add(this.workpiece);
    this.placementMatrix.decompose(this.workpiece.position, this.workpiece.quaternion, this.workpiece.scale);
    this.scene.updateMatrixWorld(true);
    return this.workpiece;
  }

  getCalibrationPoints() {
    this.scene.updateMatrixWorld(true);
    return Object.fromEntries(Object.entries(this.markers).map(([name, node]) => [name, node.getWorldPosition(new THREE.Vector3())]));
  }
}
