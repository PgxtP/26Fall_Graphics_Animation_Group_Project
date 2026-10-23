import './teachingBox.css';

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
    if (!container || typeof container.append !== 'function' || !container.ownerDocument) {
      throw new TypeError('container 必须是可挂载内容的 DOM 节点');
    }
    if (!Array.isArray(jointLimits) || jointLimits.length !== 6 || !jointLimits.every(limit => (
      limit
      && Number.isFinite(limit.min)
      && Number.isFinite(limit.max)
      && limit.min <= limit.max
    ))) {
      throw new TypeError('jointLimits 必须包含 6 组有效的弧度上下限');
    }

    this.container = container;
    this.jointLimits = jointLimits.map(limit => ({ ...limit }));
    this.speedRatio = 1.0;
    this.state = 'ENABLED';
    this.jointChangeCallback = undefined;
    this.stateChangeCallback = undefined;
    this.jointAnglesRad = this.jointLimits.map(limit => this.#clamp(0, limit.min, limit.max));
    this.jointControls = [];

    this.#createJointPanel();
  }

  #createJointPanel() {
    const document = this.container.ownerDocument;
    const panel = document.createElement('section');
    panel.className = 'teaching-box';
    panel.dataset.state = this.state;
    panel.setAttribute('aria-label', '虚拟示教器');

    const panelHeader = document.createElement('div');
    panelHeader.className = 'teaching-box__header';

    const heading = document.createElement('h2');
    heading.className = 'teaching-box__title';
    heading.textContent = '虚拟示教器';

    const stateBadge = document.createElement('span');
    stateBadge.className = 'teaching-box__state';
    stateBadge.setAttribute('aria-live', 'polite');
    panelHeader.append(heading, stateBadge);

    const jointHeading = document.createElement('h3');
    jointHeading.className = 'teaching-box__section-title';
    jointHeading.textContent = '关节控制';

    const jointSection = document.createElement('div');
    jointSection.className = 'teaching-box__joints';

    this.jointLimits.forEach((limit, index) => {
      const minDeg = this.#radToDeg(limit.min);
      const maxDeg = this.#radToDeg(limit.max);
      const initialDeg = this.#radToDeg(this.jointAnglesRad[index]);

      const row = document.createElement('div');
      row.className = 'joint-control';

      const header = document.createElement('div');
      header.className = 'joint-control__header';

      const name = document.createElement('span');
      name.className = 'joint-control__name';
      name.textContent = `J${index + 1}`;

      const limitText = document.createElement('span');
      limitText.className = 'joint-control__limit';
      limitText.textContent = `${this.#formatDegrees(minDeg)}° ～ ${this.#formatDegrees(maxDeg)}°`;
      header.append(name, limitText);

      const fields = document.createElement('div');
      fields.className = 'joint-control__fields';

      const slider = document.createElement('input');
      slider.className = 'joint-control__slider';
      slider.type = 'range';
      slider.min = String(minDeg);
      slider.max = String(maxDeg);
      slider.step = '0.1';
      slider.value = String(initialDeg);
      slider.setAttribute('aria-label', `J${index + 1} 关节角度滑块`);

      const numberField = document.createElement('label');
      numberField.className = 'joint-control__number-field';

      const numberInput = document.createElement('input');
      numberInput.className = 'joint-control__number';
      numberInput.type = 'number';
      numberInput.min = String(minDeg);
      numberInput.max = String(maxDeg);
      numberInput.step = '0.1';
      numberInput.value = this.#formatDegrees(initialDeg);
      numberInput.setAttribute('aria-label', `J${index + 1} 关节角度数值`);

      const unit = document.createElement('span');
      unit.className = 'joint-control__unit';
      unit.textContent = '°';
      numberField.append(numberInput, unit);
      fields.append(slider, numberField);
      row.append(header, fields);
      jointSection.append(row);

      slider.addEventListener('input', () => {
        this.#setJointDegrees(index, Number(slider.value), true);
      });
      numberInput.addEventListener('input', () => {
        const value = Number(numberInput.value);
        if (numberInput.value !== '' && Number.isFinite(value)) {
          this.#setJointDegrees(index, value, true);
        }
      });
      numberInput.addEventListener('change', () => {
        const value = Number(numberInput.value);
        if (numberInput.value === '' || !Number.isFinite(value)) {
          this.#renderJoint(index);
        }
      });

      this.jointControls.push({ slider, numberInput });
    });

    const safetySection = document.createElement('section');
    safetySection.className = 'teaching-box__section';

    const safetyHeading = document.createElement('h3');
    safetyHeading.className = 'teaching-box__section-title';
    safetyHeading.textContent = '安全控制';

    const safetyButtons = document.createElement('div');
    safetyButtons.className = 'teaching-box__button-row';

    const enableButton = document.createElement('button');
    enableButton.type = 'button';
    enableButton.className = 'teaching-box__button';

    const estopButton = document.createElement('button');
    estopButton.type = 'button';
    estopButton.className = 'teaching-box__button teaching-box__button--danger';
    estopButton.textContent = '急停';

    const resetButton = document.createElement('button');
    resetButton.type = 'button';
    resetButton.className = 'teaching-box__button';
    resetButton.textContent = '复位急停';
    resetButton.hidden = true;
    safetyButtons.append(enableButton, estopButton, resetButton);

    const stateMessage = document.createElement('p');
    stateMessage.className = 'teaching-box__state-message';
    stateMessage.setAttribute('role', 'status');

    const speedControl = document.createElement('label');
    speedControl.className = 'teaching-box__speed';

    const speedLabel = document.createElement('span');
    speedLabel.textContent = '速度倍率';

    const speedSlider = document.createElement('input');
    speedSlider.type = 'range';
    speedSlider.min = '10';
    speedSlider.max = '100';
    speedSlider.step = '10';
    speedSlider.value = '100';
    speedSlider.setAttribute('aria-label', '速度倍率');

    const speedValue = document.createElement('output');
    speedValue.className = 'teaching-box__speed-value';
    speedValue.textContent = '100%';
    speedControl.append(speedLabel, speedSlider, speedValue);
    safetySection.append(safetyHeading, safetyButtons, stateMessage, speedControl);

    const tcpSection = document.createElement('section');
    tcpSection.className = 'teaching-box__section';

    const tcpHeading = document.createElement('h3');
    tcpHeading.className = 'teaching-box__section-title';
    tcpHeading.textContent = 'TCP 世界位姿';

    const positionGrid = document.createElement('div');
    positionGrid.className = 'teaching-box__position-grid';
    const positionFields = {};
    ['x', 'y', 'z'].forEach(axis => {
      const item = document.createElement('div');
      const label = document.createElement('span');
      label.textContent = `${axis.toUpperCase()} / m`;
      const value = document.createElement('strong');
      value.textContent = '—';
      item.append(label, value);
      positionGrid.append(item);
      positionFields[axis] = value;
    });

    const poseDetails = document.createElement('dl');
    poseDetails.className = 'teaching-box__pose-details';

    const createPoseRow = (labelText) => {
      const row = document.createElement('div');
      const label = document.createElement('dt');
      label.textContent = labelText;
      const value = document.createElement('dd');
      value.textContent = '—';
      row.append(label, value);
      poseDetails.append(row);
      return value;
    };
    const rpyField = createPoseRow('RPY / ZYX / °');
    const quaternionField = createPoseRow('四元数 / xyzw');
    tcpSection.append(tcpHeading, positionGrid, poseDetails);

    panel.append(panelHeader, jointHeading, jointSection, safetySection, tcpSection);
    this.container.append(panel);
    this.element = panel;
    this.stateBadge = stateBadge;
    this.enableButton = enableButton;
    this.estopButton = estopButton;
    this.resetButton = resetButton;
    this.stateMessage = stateMessage;
    this.speedSlider = speedSlider;
    this.speedValue = speedValue;
    this.tcpFields = { position: positionFields, rpy: rpyField, quaternion: quaternionField };

    enableButton.addEventListener('click', () => {
      this.#transitionTo(this.state === 'ENABLED' ? 'DISABLED' : 'ENABLED');
    });
    estopButton.addEventListener('click', () => this.#transitionTo('ESTOP'));
    resetButton.addEventListener('click', () => {
      if (this.state === 'ESTOP') this.#transitionTo('DISABLED');
    });
    speedSlider.addEventListener('input', () => {
      this.speedRatio = Number(speedSlider.value) / 100;
      speedValue.textContent = `${speedSlider.value}%`;
    });

    this.#renderState();
  }

  #transitionTo(nextState) {
    if (!['ENABLED', 'DISABLED', 'ESTOP'].includes(nextState)) {
      throw new TypeError('未知示教器状态');
    }
    if (this.state === 'ESTOP' && nextState === 'ENABLED') return;
    if (this.state === nextState) return;
    this.state = nextState;
    this.#renderState();
    if (typeof this.stateChangeCallback === 'function') {
      this.stateChangeCallback(nextState);
    }
  }

  #renderState() {
    const labels = {
      ENABLED: '已使能',
      DISABLED: '已失能',
      ESTOP: '急停已锁定',
    };
    const messages = {
      ENABLED: '关节点动已允许',
      DISABLED: '关节点动已禁止',
      ESTOP: '所有运动已冻结；复位后仍需重新使能',
    };
    const controlsDisabled = this.state !== 'ENABLED';
    this.element.dataset.state = this.state;
    this.stateBadge.textContent = labels[this.state];
    this.stateMessage.textContent = messages[this.state];
    this.enableButton.textContent = this.state === 'ENABLED' ? '失能' : '使能';
    this.enableButton.disabled = this.state === 'ESTOP';
    this.estopButton.disabled = this.state === 'ESTOP';
    this.resetButton.hidden = this.state !== 'ESTOP';
    this.jointControls.forEach(({ slider, numberInput }) => {
      slider.disabled = controlsDisabled;
      numberInput.disabled = controlsDisabled;
    });
  }

  #setJointDegrees(index, degrees, emitChange) {
    const limit = this.jointLimits[index];
    const clampedDegrees = this.#clamp(
      degrees,
      this.#radToDeg(limit.min),
      this.#radToDeg(limit.max),
    );
    this.jointAnglesRad[index] = this.#clamp(
      this.#degToRad(clampedDegrees),
      limit.min,
      limit.max,
    );
    this.#renderJoint(index);

    if (emitChange && typeof this.jointChangeCallback === 'function') {
      this.jointChangeCallback([...this.jointAnglesRad]);
    }
  }

  #renderJoint(index) {
    const degrees = this.#radToDeg(this.jointAnglesRad[index]);
    const { slider, numberInput } = this.jointControls[index];
    slider.value = String(degrees);
    numberInput.value = this.#formatDegrees(degrees);
  }

  #clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  #radToDeg(radians) {
    const degrees = radians * 180 / Math.PI;
    return Math.round(degrees * 1e10) / 1e10;
  }

  #degToRad(degrees) {
    return degrees * Math.PI / 180;
  }

  #formatDegrees(degrees) {
    const normalized = Math.abs(degrees) < 0.00005 ? 0 : degrees;
    return Number(normalized.toFixed(4)).toString();
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
   * 主程序状态同步：更新示教器显示，但不重复触发状态回调。
   * @param {'ENABLED' | 'DISABLED' | 'ESTOP'} state
   */
  syncState(state) {
    if (!['ENABLED', 'DISABLED', 'ESTOP'].includes(state)) {
      throw new TypeError('未知示教器状态');
    }
    this.state = state;
    this.#renderState();
  }

  /**
   * 每帧刷新屏幕 HUD 上的末端位姿显示
   * @param {{ position: { x: number, y: number, z: number }, rotationRPY: { roll: number, pitch: number, yaw: number }, quaternion: { x: number, y: number, z: number, w: number } }} pose
   */
  updateTCPDisplay(pose) {
    const values = [
      pose?.position?.x, pose?.position?.y, pose?.position?.z,
      pose?.rotationRPY?.roll, pose?.rotationRPY?.pitch, pose?.rotationRPY?.yaw,
      pose?.quaternion?.x, pose?.quaternion?.y, pose?.quaternion?.z, pose?.quaternion?.w,
    ];
    if (!values.every(Number.isFinite)) {
      throw new TypeError('TCP 位姿必须包含有限的位置、RPY 和四元数分量');
    }

    ['x', 'y', 'z'].forEach(axis => {
      this.tcpFields.position[axis].textContent = pose.position[axis].toFixed(4);
    });
    this.tcpFields.rpy.textContent = [
      pose.rotationRPY.roll,
      pose.rotationRPY.pitch,
      pose.rotationRPY.yaw,
    ].map(value => this.#radToDeg(value).toFixed(2)).join(' / ');
    this.tcpFields.quaternion.textContent = [
      pose.quaternion.x,
      pose.quaternion.y,
      pose.quaternion.z,
      pose.quaternion.w,
    ].map(value => value.toFixed(4)).join(' / ');
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
    if (!Array.isArray(anglesRad) || anglesRad.length !== 6 || !anglesRad.every(Number.isFinite)) {
      throw new TypeError('anglesRad 必须是包含 6 个有限弧度值的数组');
    }
    anglesRad.forEach((angle, index) => {
      const limit = this.jointLimits[index];
      this.jointAnglesRad[index] = this.#clamp(angle, limit.min, limit.max);
      this.#renderJoint(index);
    });
  }
}
