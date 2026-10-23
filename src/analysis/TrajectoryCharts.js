/**
 * 【5号位负责】TrajectoryCharts 类：负责高频采样末端轨迹数据、绘制位置/速度曲线与导出 CSV
 */
export class TrajectoryCharts {
  constructor() {
    this.samples = [];
  }

  /**
   * 运动中高频记录数据点（由动画循环定期调用）
   * @param {number} timestamp - 当前仿真运行时间 (秒)
   * @param {{ x: number, y: number, z: number }} tcpPosition - TCP 世界坐标 (米)
   * @param {number} linearVelocity - 当前末端线速度大小 (米/秒)
   */
  recordSample(timestamp, tcpPosition, linearVelocity) {
    this.samples.push({
      t: timestamp,
      x: tcpPosition.x,
      y: tcpPosition.y,
      z: tcpPosition.z,
      v: linearVelocity,
    });
  }

  /**
   * 渲染动态图表（在网页抽屉或面板中展示 3D 轨迹图、位置曲线 p(t) 与速度曲线 v(t)）
   * @param {HTMLElement} chartContainer - 图表挂载的 DOM 节点
   */
  renderCharts(chartContainer) {
    // TODO (5号位): 使用 Canvas / Chart.js / ECharts 绘制位置与速度随时间变化曲线
  }

  /**
   * 导出采样数据为 CSV 格式文本（用于撰写报告时作图或定量分析）
   * @returns {string} - CSV 内容字符串: "t,x,y,z,v\n0.00,0.50,..."
   */
  exportToCSV() {
    const header = 't,x,y,z,v';
    const rows = this.samples.map(
      (s) => `${s.t.toFixed(4)},${s.x.toFixed(6)},${s.y.toFixed(6)},${s.z.toFixed(6)},${s.v.toFixed(6)}`,
    );
    return [header, ...rows].join('\n');
  }
}
