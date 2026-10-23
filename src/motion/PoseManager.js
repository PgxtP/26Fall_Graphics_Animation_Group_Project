/**
 * 【4号位负责】PoseManager 类：负责示教位姿点记录、删除与 JSON 导出（至少记录 4 个位姿）
 */
export class PoseManager {
  constructor() {
    this.poses = [];
  }

  /**
   * 保存当前位姿为一个示教点（至少存 4 个点构成连续程序）
   * @param {string} name - 示教点名称（如 "P1_Pick", "P2_Lift"）
   * @param {number[]} anglesRad - 当前 6 关节弧度数组
   * @returns {{ id: string, name: string, angles: number[] }}
   */
  savePose(name, anglesRad) {
    // TODO (4号位): 记录位姿点并返回对象
    const pose = {
      id: `pose_${Date.now()}`,
      name,
      angles: [...anglesRad],
    };
    this.poses.push(pose);
    return pose;
  }

  /**
   * 获取所有已存示教点列表
   * @returns {Array<{ id: string, name: string, angles: number[] }>}
   */
  getPoseList() {
    return [...this.poses];
  }

  /**
   * 导出为 JSON 字符串（供下载或在报告中展示程序清单）
   * @returns {string}
   */
  exportProgramJSON() {
    return JSON.stringify(this.poses, null, 2);
  }
}
