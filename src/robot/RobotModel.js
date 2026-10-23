import * as THREE from 'three';
import { ROBOT_CONFIG } from './robot3Config.js';
import { validateAngles, validateRigidOffset } from './Kinematics.js';

/** 输入为相对 GLB 导出零位的弧度。保留全部初始平移和旋转，不直接覆盖原始欧拉角。 */
export class RobotModel {
  constructor(gltfScene, jointLimits = ROBOT_CONFIG.jointLimits) {
    if (!gltfScene?.isObject3D) throw new TypeError('必须传入 GLTFLoader 返回的场景根节点');
    if (!Array.isArray(jointLimits) || jointLimits.length !== 6 || !Array.from(jointLimits).every(l => l && Number.isFinite(l.min) && Number.isFinite(l.max) && l.min <= 0 && l.max >= 0)) throw new TypeError('需要 6 组有限且包含零位的关节限位');
    this.root = gltfScene;
    this.jointLimits = jointLimits.map(l => ({ ...l }));
    this.currentAngles = [0, 0, 0, 0, 0, 0];
    this.root.traverse(node => node.quaternion.normalize());
    this.joints = Array.from({ length: 6 }, (_, i) => {
      const matches = [];
      this.root.traverse(node => { if (node.name === 'J' + (i + 1)) matches.push(node); });
      if (matches.length !== 1) throw new Error('模型必须包含唯一的 J' + (i + 1) + ' 节点');
      return matches[0];
    });
    this.flange = this.root.getObjectByName('link_flange');
    this.tcpNode = this.root.getObjectByName('tcp');
    if (!this.flange) throw new Error('缺少 link_flange 法兰节点');
    const path = [];
    for (let node = this.flange; node; node = node.parent) {
      path.unshift(node);
      if (node === this.root) break;
    }
    const indices = this.joints.map(j => path.indexOf(j));
    if (!path.includes(this.root) || indices.some((index, i) => index < 0 || (i > 0 && index <= indices[i - 1]))) throw new Error('J1~J6 必须按顺序处于同一法兰祖先链');
    this.zeroQuaternions = this.joints.map(node => node.quaternion.clone());
    this.axes = this.joints.map((_, i) => new THREE.Vector3().fromArray(ROBOT_CONFIG.chain.find(n => n.jointIndex === i).axis).normalize());
    this.chainNodes = path;
    this.restChain = path.map(node => ({
      name: node.name, translation: node.position.toArray(), quaternion: node.quaternion.toArray(),
      scale: node.scale.toArray(), jointIndex: this.joints.indexOf(node),
      ...(this.joints.includes(node) ? { axis: this.axes[this.joints.indexOf(node)].toArray() } : {}),
    }));
    this.root.updateWorldMatrix(true, true);
  }

  setJointAngles(anglesRad) {
    try { validateAngles(anglesRad); } catch (error) {
      return { success: false, clampedAngles: this.getJointAngles(), errorMsg: error.message };
    }
    const clampedAngles = anglesRad.map((angle, i) => THREE.MathUtils.clamp(angle, this.jointLimits[i].min, this.jointLimits[i].max));
    this.joints.forEach((node, i) => {
      // 轴在父坐标系：R_motion * R_zero，不能交换乘法顺序。
      node.quaternion.setFromAxisAngle(this.axes[i], clampedAngles[i]).multiply(this.zeroQuaternions[i]).normalize();
    });
    this.currentAngles = clampedAngles;
    this.root.updateWorldMatrix(true, true);
    return { success: true, clampedAngles: [...clampedAngles],
      ...(clampedAngles.some((a, i) => a !== anglesRad[i]) ? { errorMsg: '超出软限位的关节已钳制到边界' } : {}),
    };
  }

  getJointAngles() { return [...this.currentAngles]; }

  /** 默认法兰原点；工具偏置用米。RPY 定义为 Rz(yaw) Ry(pitch) Rx(roll)，即 ZYX。 */
  getTCPPose(toolOffsetMatrix) {
    this.flange.updateWorldMatrix(true, false);
    const matrixWorld = this.flange.matrixWorld.clone();
    if (toolOffsetMatrix) { validateRigidOffset(toolOffsetMatrix); matrixWorld.multiply(toolOffsetMatrix); }
    const position = new THREE.Vector3(), quaternion = new THREE.Quaternion(), scale = new THREE.Vector3();
    matrixWorld.decompose(position, quaternion, scale);
    quaternion.normalize();
    const rpy = new THREE.Euler().setFromQuaternion(quaternion, 'ZYX');
    return { position, rotationRPY: { roll: rpy.x, pitch: rpy.y, yaw: rpy.z }, quaternion, matrixWorld };
  }

  /** FK 只保存数值，之后不会触碰模型。重新摆放模型后需重新取得配置。 */
  getKinematicConfig() {
    this.root.updateWorldMatrix(true, false);
    return {
      baseMatrix: (this.root.parent?.matrixWorld ?? new THREE.Matrix4()).toArray(),
      chain: this.restChain.map((rest, i) => ({
        ...structuredClone(rest),
        translation: this.chainNodes[i].position.toArray(),
        quaternion: rest.jointIndex < 0 ? this.chainNodes[i].quaternion.toArray() : [...rest.quaternion],
        scale: this.chainNodes[i].scale.toArray(),
      })),
    };
  }
}
