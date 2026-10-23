import * as THREE from 'three';

/**
 * 【3号位负责】Workstation 类：负责工业工作台场景搭建与工件抓取/释放父子层级切换
 */
export class Workstation {
  /**
   * 在主场景中添加工业工作台、参考地网与待抓取的工件箱
   * @param {THREE.Scene} scene - 主场景
   * @returns {{ table: THREE.Mesh, workpiece: THREE.Mesh }}
   */
  static initWorkstation(scene) {
    // TODO (3号位): 创建工作台 Mesh 与待抓取物块 Mesh 并挂入 scene
    const table = new THREE.Mesh();
    const workpiece = new THREE.Mesh();
    return { table, workpiece };
  }

  /**
   * 执行吸附/抓取工件（动态切换物块父节点为机械臂末端连杆，保持世界位姿不跳变）
   * @param {THREE.Object3D} toolFlange - 机械臂末端节点
   * @param {THREE.Object3D} workpiece - 待抓取物块
   */
  static attachWorkpiece(toolFlange, workpiece) {
    // TODO (3号位): 使用 toolFlange.attach(workpiece) 实现抓取吸附
  }

  /**
   * 释放工件（将物块父节点放回主场景世界，并保持当前世界位置）
   * @param {THREE.Scene} scene - 主场景
   * @param {THREE.Object3D} workpiece - 物块
   */
  static detachWorkpiece(scene, workpiece) {
    // TODO (3号位): 使用 scene.attach(workpiece) 实现释放
  }
}
