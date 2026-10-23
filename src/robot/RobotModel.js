import * as THREE from 'three';

/**
 * 【1号位负责】RobotModel 类：负责 Three.js 骨骼层级驱动与末端位姿提取
 */
export class RobotModel {
  /**
   * 构造函数：解析 gltf 场景树，提取并建立 J1~J6 的父子层级引用
   * @param {THREE.Object3D} gltfScene - GLTFLoader 载入的根模型对象
   * @param {Array<{ min: number, max: number }>} jointLimits - 6 个关节的弧度软限位区间
   */
  constructor(gltfScene, jointLimits = []) {
    this.root = gltfScene;
    this.jointLimits = jointLimits;
    this.joints = [];
    this.currentAngles = [0, 0, 0, 0, 0, 0];
    // TODO (1号位): 遍历 gltfScene 提取关节节点并记录初始零位四元数 qZero
  }

  /**
   * 驱动机械臂 6 个关节旋转（内部使用弧度制，先做限位钳制再旋转）
   * @param {number[]} anglesRad - 6 个关节的目标角度（弧度制）
   * @returns {{ success: boolean, clampedAngles: number[], errorMsg?: string }}
   */
  setJointAngles(anglesRad) {
    // TODO (1号位): 对每个关节角进行限位 clamp，并更新对应节点的局部旋转
    return {
      success: true,
      clampedAngles: [...anglesRad],
    };
  }

  /**
   * 获取当前机械臂实际关节角度
   * @returns {number[]} - 长度为 6 的弧度数组 [j1, j2, j3, j4, j5, j6]
   */
  getJointAngles() {
    return [...this.currentAngles];
  }

  /**
   * 获取末端工具中心点 (TCP) 当前的世界位姿
   * @param {THREE.Matrix4} [toolOffsetMatrix] - 可选的工具偏置矩阵（默认为无偏置的法兰盘）
   * @returns {{
   *   position: THREE.Vector3,
   *   rotationRPY: { roll: number, pitch: number, yaw: number },
   *   quaternion: THREE.Quaternion,
   *   matrixWorld: THREE.Matrix4
   * }}
   */
  getTCPPose(toolOffsetMatrix) {
    // TODO (1号位): 从末端节点读取世界矩阵 matrixWorld，叠加 toolOffsetMatrix 后提取位置、四元数与 RPY 欧拉角
    return {
      position: new THREE.Vector3(),
      rotationRPY: { roll: 0, pitch: 0, yaw: 0 },
      quaternion: new THREE.Quaternion(),
      matrixWorld: new THREE.Matrix4(),
    };
  }
}
