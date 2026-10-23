import * as THREE from 'three';

/**
 * 【3号位负责】Calibration 类：负责工具坐标系 (TCP) 标定与用户工件坐标系三点法标定
 */
export class Calibration {
  /**
   * 用户工件坐标系三点法标定算法（施密特正交化）
   * @param {THREE.Vector3} pOrigin - 工件原点 O 的世界坐标 (测得的 TCP 点 1)
   * @param {THREE.Vector3} pXAxis  - 工件 X 轴正方向点 Px (测得的 TCP 点 2)
   * @param {THREE.Vector3} pXYPlane - 工件 XY 平面上 Y>0 侧点 Pxy (测得的 TCP 点 3)
   * @returns {{
   *   userMatrix: THREE.Matrix4,
   *   origin: THREE.Vector3,
   *   xAxis: THREE.Vector3,
   *   yAxis: THREE.Vector3,
   *   zAxis: THREE.Vector3
   * }}
   */
  static calibrateUserFrameThreePoints(pOrigin, pXAxis, pXYPlane) {
    // TODO (3号位): 用三点法构建正交右手基向量 Ux, Uy, Uz 并组装 4x4 齐次变换矩阵
    return {
      userMatrix: new THREE.Matrix4(),
      origin: pOrigin.clone(),
      xAxis: new THREE.Vector3(1, 0, 0),
      yAxis: new THREE.Vector3(0, 1, 0),
      zAxis: new THREE.Vector3(0, 0, 1),
    };
  }

  /**
   * 将世界坐标系下的点转换为用户工件坐标系下的相对坐标
   * @param {THREE.Vector3} pointWorld - 世界坐标
   * @param {THREE.Matrix4} userMatrix - 三点法解出的用户坐标系矩阵
   * @returns {THREE.Vector3} - 工件坐标系下的相对坐标
   */
  static worldToUserFrame(pointWorld, userMatrix) {
    // TODO (3号位): 求 userMatrix 的逆矩阵并左乘 pointWorld
    return pointWorld.clone();
  }

  /**
   * 创建工具坐标系 TCP 偏置矩阵
   * @param {THREE.Vector3} translation - 工具中心点相对法兰盘的平移偏置 (米)
   * @param {THREE.Euler} rotation - 工具相对法兰盘的朝向旋转 (弧度)
   * @returns {THREE.Matrix4} - 4x4 工具偏置矩阵
   */
  static createTCPOffset(translation, rotation) {
    // TODO (3号位): 组合平移向量与欧拉角生成 4x4 偏置矩阵
    return new THREE.Matrix4();
  }
}
