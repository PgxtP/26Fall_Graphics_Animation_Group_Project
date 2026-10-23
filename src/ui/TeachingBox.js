/**
 * 【2号位负责】TeachingBox 类：负责示教器交互面板（滑块、角度输入、使能、急停、速度倍率与位姿 HUD）
 * 注意：界面上显示【角度 (°)】与【毫米 (mm) / 米 (m)】，对外回调一律传出【弧度 (rad)】
 */
export class TeachingBox {
  /**
   * 构造函数：在页面生成示教器面板
   * @param {HTMLElement} container - 挂载的 DOM 节点
   * @param {Array<{ min: number, max: number }>} jointLimits - 各关节弧度限位
   */
  constructor(container, jointLimits = []) {
    this.container = container;
    this.jointLimits = jointLimits;
    this.speedRatio = 1.0;
    this.state = 'ENABLED';
    // TODO (2号位): 初始化示教器 DOM 结构与事件监听
  }

  /**
   * 注册关节角度发生变动的事件回调（滑块拖拽或输入数值时触发）
   * @param {(anglesRad: number[]) => void} callback - 传递换算好的 6 关节弧度数组
   */
  onJointChange(callback) {
    this.jointChangeCallback = callback;
  }

  /**
   * 注册急停 (E-Stop) 与使能状态变更事件
   * @param {(state: 'ENABLED' | 'DISABLED' | 'ESTOP') => void} callback
   */
  onStateChange(callback) {
    this.stateChangeCallback = callback;
  }

  /**
   * 每帧刷新屏幕 HUD 上的末端位姿显示
   * @param {{ position: { x: number, y: number, z: number }, rotationRPY: { roll: number, pitch: number, yaw: number }, quaternion: { x: number, y: number, z: number, w: number } }} pose
   */
  updateTCPDisplay(pose) {
    // TODO (2号位): 将位置与姿态渲染到示教器面板 HUD 上
  }

  /**
   * 获取当前用户设定的速度倍率
   * @returns {number} - 0.1 到 1.0 之间的浮点数
   */
  getSpeedRatio() {
    return this.speedRatio;
  }

  /**
   * 外部同步：当机械臂执行 MOVJ 连续自动运动时，反向更新示教器滑块的显示数值
   * @param {number[]} anglesRad - 当前各关节弧度
   */
  syncSliders(anglesRad) {
    // TODO (2号位): 将弧度转为角度并刷新滑块与输入框位置
  }
}
