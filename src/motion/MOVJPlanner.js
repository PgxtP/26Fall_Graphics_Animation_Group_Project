/**
 * 【4号位负责】MOVJPlanner 类：负责关节空间连续轨迹插值与播放状态机
 */
export class MOVJPlanner {
  constructor() {
    this.program = [];
    this.isMoving = false;
    this.speedRatio = 1.0;
    this.loop = false;
    this.currentSegment = 0;
    this.progress = 0;
    this.currentVelocity = 0;
  }

  /**
   * 加载待执行的连续示教程序
   * @param {Array<{ name: string, angles: number[] }>} program - 至少 4 个连续 Pose
   */
  loadProgram(program) {
    this.program = program || [];
  }

  /**
   * 启动连续执行播放
   * @param {number} speedRatio - 速度倍率 (0.1 ~ 1.0)
   * @param {boolean} loop - 是否循环播放
   */
  play(speedRatio = 1.0, loop = false) {
    this.speedRatio = speedRatio;
    this.loop = loop;
    this.isMoving = this.program.length >= 2;
    this.currentSegment = 0;
    this.progress = 0;
  }

  /**
   * 每帧在 requestAnimationFrame 动画循环中被调用
   * @param {number} deltaTime - 相邻两帧的真实时间差 dt (秒)
   * @returns {{
   *   currentAngles: number[],
   *   isMoving: boolean,
   *   currentSegment: number,
   *   progress: number
   * }}
   */
  update(deltaTime) {
    // TODO (4号位): 根据 deltaTime 与 speedRatio 进行关节空间插值（LERP 或加减速平滑插值）
    return {
      currentAngles: [0, 0, 0, 0, 0, 0],
      isMoving: this.isMoving,
      currentSegment: this.currentSegment,
      progress: this.progress,
    };
  }

  pause() {
    this.isMoving = false;
  }

  resume() {
    if (this.program.length >= 2) {
      this.isMoving = true;
    }
  }

  stop() {
    this.isMoving = false;
    this.currentSegment = 0;
    this.progress = 0;
  }
}
