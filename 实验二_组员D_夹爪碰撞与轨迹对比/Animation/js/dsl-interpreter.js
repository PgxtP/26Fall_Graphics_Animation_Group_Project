import { planMOVLTrajectory, poseFromMatrix } from './movl-planner.js';

export const DSL_STATUS = Object.freeze({
  IDLE: 'IDLE',
  RUNNING: 'RUNNING',
  PAUSED: 'PAUSED',
  ESTOP: 'E-STOP',
  DONE: 'DONE',
});

export const DSL_LIMITS = Object.freeze({
  maxSteps: 500, // 展开 LOOP 之后的最大指令条数
  maxLoopCount: 99, // LOOP 的最大重复次数
  maxWaitSeconds: 60, // 单次 WAIT 的最大秒数
  minSpeed: 1,
  maxSpeed: 100,
});

const POSE_PATTERN = /^P[1-9]\d*$/i;

/** 去掉行内注释，但保留字符串外的内容（DSL 里不需要引号，直接截断即可）。 */
function stripComment(raw) {
  const index = raw.search(/#|;|\/\//);
  return index >= 0 ? raw.slice(0, index) : raw;
}

function normalizePoseMap(poseMap) {
  const map = new Map();
  if (!poseMap) return map;
  if (poseMap instanceof Map) {
    for (const [name, pose] of poseMap) map.set(String(name).toUpperCase(), pose);
    return map;
  }
  if (Array.isArray(poseMap)) {
    for (const pose of poseMap) if (pose?.name) map.set(String(pose.name).toUpperCase(), pose);
    return map;
  }
  if (typeof poseMap === 'object') {
    for (const [name, pose] of Object.entries(poseMap)) map.set(String(name).toUpperCase(), pose);
  }
  return map;
}

/**
 * 把 DSL 文本解析成可执行的指令数组（LOOP 在解析期展开，界面能逐条高亮当前行）。
 *
 * @param {string} scriptText DSL 源码
 * @param {Map|Array|Object} poseMap 已记录位姿：name → { angles:number[5] }
 * @returns {{ok:boolean, errors:Array<{line:number,message:string}>, instructions:Array, poseNames:string[], rawLineCount:number}}
 */
export function parseDSLScript(scriptText, poseMap) {
  const errors = [];
  const poses = normalizePoseMap(poseMap);
  const text = String(scriptText ?? '');
  const rawLines = text.split(/\r?\n/);
  const tokens = [];

  const fail = (line, message) => errors.push({ line, message: `第 ${line} 行：${message}` });

  rawLines.forEach((raw, index) => {
    const line = index + 1;
    const content = stripComment(raw).trim();
    if (!content) return;
    let match;

    if ((match = /^(MOVJ|MOVL)\s+(\S+)$/i.exec(content))) {
      const name = match[2].toUpperCase();
      if (!POSE_PATTERN.test(name)) { fail(line, `位姿名必须写成 P1、P2…，收到 ${match[2]}`); return; }
      if (!poses.has(name)) { fail(line, `引用了未记录的位姿 ${name}，请先记录或导入该位姿`); return; }
      const pose = poses.get(name);
      if (!Array.isArray(pose?.angles) || pose.angles.length !== 5 || pose.angles.some((v) => !Number.isFinite(v))) {
        fail(line, `位姿 ${name} 的关节角无效（需要 5 个弧度值）`); return;
      }
      tokens.push({ kind: 'instruction', type: match[1].toUpperCase(), name, angles: [...pose.angles], line });
      return;
    }
    if ((match = /^GRIP\s+(ON|OFF|CLOSE|OPEN|1|0)$/i.exec(content))) {
      const word = match[1].toUpperCase();
      const value = word === 'ON' || word === 'CLOSE' || word === '1' ? 1 : 0;
      tokens.push({ kind: 'instruction', type: 'GRIP', value, line });
      return;
    }
    if ((match = /^WAIT\s+(\d+(?:\.\d+)?)$/i.exec(content))) {
      const seconds = Number(match[1]);
      if (!(seconds >= 0) || seconds > DSL_LIMITS.maxWaitSeconds) {
        fail(line, `WAIT 秒数需在 0～${DSL_LIMITS.maxWaitSeconds} 之间，收到 ${match[1]}`); return;
      }
      tokens.push({ kind: 'instruction', type: 'WAIT', seconds, line });
      return;
    }
    if ((match = /^SET\s+SPEED\s+(\d+(?:\.\d+)?)$/i.exec(content))) {
      const percent = Number(match[1]);
      if (!(percent >= DSL_LIMITS.minSpeed) || percent > DSL_LIMITS.maxSpeed) {
        fail(line, `速度倍率需在 ${DSL_LIMITS.minSpeed}～${DSL_LIMITS.maxSpeed} 之间，收到 ${match[1]}`); return;
      }
      tokens.push({ kind: 'instruction', type: 'SPEED', percent, line });
      return;
    }
    if ((match = /^LOOP\s+(\d+)$/i.exec(content))) {
      const count = Number(match[1]);
      if (!(count >= 1) || count > DSL_LIMITS.maxLoopCount) {
        fail(line, `LOOP 次数需在 1～${DSL_LIMITS.maxLoopCount} 之间，收到 ${match[1]}`); return;
      }
      tokens.push({ kind: 'loopStart', count, line });
      return;
    }
    if (/^END$/i.test(content)) { tokens.push({ kind: 'loopEnd', line }); return; }

    fail(line, `无法识别的指令「${content}」；支持 MOVJ / MOVL / GRIP / WAIT / LOOP…END / SET SPEED`);
  });

  // 先把 LOOP…END 组织成树，再展开成扁平指令表，便于执行与高亮。
  let cursor = 0;
  const parseBlock = (expectEnd) => {
    const items = [];
    while (cursor < tokens.length) {
      const token = tokens[cursor];
      if (token.kind === 'loopEnd') {
        cursor += 1;
        if (!expectEnd) fail(token.line, 'END 没有对应的 LOOP');
        return items;
      }
      cursor += 1;
      if (token.kind === 'loopStart') {
        const body = parseBlock(true);
        if (body.length === 0) fail(token.line, 'LOOP 与 END 之间没有指令');
        items.push({ kind: 'loop', count: token.count, line: token.line, body });
        continue;
      }
      items.push(token);
    }
    if (expectEnd) {
      const lastLine = tokens.length ? tokens[tokens.length - 1].line : rawLines.length;
      fail(lastLine, 'LOOP 缺少配对的 END');
    }
    return items;
  };
  const tree = parseBlock(false);

  const instructions = [];
  const expand = (items) => {
    for (const item of items) {
      if (instructions.length > DSL_LIMITS.maxSteps) return;
      if (item.kind === 'loop') {
        for (let round = 0; round < item.count; round += 1) {
          expand(item.body);
          if (instructions.length > DSL_LIMITS.maxSteps) return;
        }
        continue;
      }
      instructions.push({ ...item, kind: undefined });
    }
  };
  expand(tree);
  if (instructions.length > DSL_LIMITS.maxSteps) {
    fail(0, `程序展开后超过 ${DSL_LIMITS.maxSteps} 条指令，请减少 LOOP 次数`);
  }

  return {
    ok: errors.length === 0,
    errors,
    instructions: errors.length ? [] : instructions,
    poseNames: [...new Set(instructions.filter((i) => i.name).map((i) => i.name))],
    rawLineCount: rawLines.length,
  };
}

/**
 * 创建 DSL 执行器。接口与 movj-program.js 的 createProgramRunner 保持兼容：
 * state.running / state.instructions / state.index / state.message / start() / stop() / update()。
 *
 * @param {object} api 机器人对象（joints、tcp、robot、solveIK、workstation…）
 * @param {object} motion pendant-motion 的实例（moveJ / state / allowed / busy）
 * @param {object} [env] 注入点：{ ikSolver, poseOfAngles, currentPose, grip, planner, movlSpeed }
 */
export function createDSLRunner(api, motion, env = {}) {
  const state = {
    status: DSL_STATUS.IDLE,
    running: false, // 兼容旧接口：运行时与暂停时都为 true，界面据此锁定编辑
    paused: false,
    index: -1,
    instructions: [],
    message: '尚未执行',
    progress: 0,
    speedPercent: 100,
    lastMOVL: null,
  };

  let stopVersion = null;
  let motionPending = false;
  let moving = null; // 正在执行的 MOVL 插补队列：{ samples, cursor, name, line }
  let waiting = null; // 正在执行的 WAIT：{ until }
  let savedSpeed = null;
  let savedPoseOfAngles = null;

  const clock = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());
  const setStatus = (status) => {
    state.status = status;
    state.running = status === DSL_STATUS.RUNNING || status === DSL_STATUS.PAUSED;
    state.paused = status === DSL_STATUS.PAUSED;
  };

  const restoreSpeed = () => {
    if (savedSpeed !== null) { motion.state.speed = savedSpeed; savedSpeed = null; state.speedPercent = motion.state.speed; }
  };

  const release = (status, message) => {
    setStatus(status);
    state.message = message;
    motionPending = false; moving = null; waiting = null;
    if (status !== DSL_STATUS.PAUSED) { motion.state.programActive = false; restoreSpeed(); }
  };

  /** 计算某个已记录位姿对应的 TCP 世界位姿（由界面注入，内部靠备份/恢复保证无副作用）。 */
  const poseOfAngles = (angles) => {
    const fn = env.poseOfAngles ?? savedPoseOfAngles;
    if (typeof fn !== 'function') throw new Error('缺少位姿正解适配器，无法执行 MOVL');
    return fn(angles);
  };

  const currentPose = () => {
    if (typeof env.currentPose === 'function') return env.currentPose();
    if (!api?.tcp?.matrixWorld) throw new Error('缺少 TCP 节点，无法规划 MOVL');
    api.robot?.updateMatrixWorld?.(true);
    return poseFromMatrix(api.tcp.matrixWorld.elements);
  };

  const solve = () => {
    const solver = env.ikSolver ?? api.solveIK;
    if (typeof solver !== 'function') throw new Error('缺少逆解函数 solveIK，无法执行 MOVL');
    return solver;
  };

  const grip = (value) => {
    if (typeof env.grip === 'function') return env.grip(value, api);
    // bindProgram 早于 Workstation 创建（robot-model.js:132 vs :133），所以这里必须惰性读取。
    const station = api.workstation;
    if (!station) throw new Error('工位尚未加载，无法执行 GRIP');
    if (value) station.attachWorkpiece();
    else station.detachWorkpiece();
    api.calibration?.invalidateUserFrame?.('物块位置已改变；如需当前工件坐标系，请重新读取标记并建立坐标系。');
    const el = typeof document !== 'undefined' ? document.getElementById('workstation-status') : null;
    if (el) el.textContent = value ? '物块已瞬移到当前 TCP 并挂载，点动后随法兰运动' : '物块已解除挂载，放回工作台放置点';
  };

  const describe = (instruction) => {
    switch (instruction.type) {
      case 'MOVJ': return `MOVJ ${instruction.name}`;
      case 'MOVL': return `MOVL ${instruction.name}`;
      case 'GRIP': return `GRIP ${instruction.value}`;
      case 'WAIT': return `WAIT ${instruction.seconds}s`;
      case 'SPEED': return `SET SPEED ${instruction.percent}%`;
      default: return instruction.type;
    }
  };

  const position = () => `（${state.index + 1}/${state.instructions.length}）`;

  /** 执行下一条指令。互锁动作（MOVJ / MOVL）只下发目标，实际推进在 update() 里做。 */
  const next = () => {
    if (!state.running) return;
    state.index += 1;
    if (state.index >= state.instructions.length) {
      const total = state.instructions.length;
      release(DSL_STATUS.DONE, `程序执行完成，共 ${total} 条指令`);
      state.progress = 1;
      return;
    }
    state.progress = state.instructions.length ? state.index / state.instructions.length : 0;
    const instruction = state.instructions[state.index];
    try {
      switch (instruction.type) {
        case 'MOVJ':
          motion.moveJ(instruction.angles);
          motionPending = true;
          state.message = `执行第 ${instruction.line} 行：MOVJ ${instruction.name}${position()}`;
          return;
        case 'MOVL': {
          const start = { ...currentPose(), joints: api.joints.map((joint) => joint.angle) };
          const end = poseOfAngles(instruction.angles);
          const baseSpeed = Number(env.movlSpeed) > 0 ? Number(env.movlSpeed) : 1; // 资产单位/秒
          const speed = baseSpeed * state.speedPercent / 100;
          const plan = planMOVLTrajectory(start, end, speed, solve(), env.planner ?? {});
          state.lastMOVL = plan;
          if (!plan.ok) {
            release(DSL_STATUS.IDLE, `MOVL ${instruction.name} 规划失败，程序已中断：${plan.reason}`);
            return;
          }
          moving = { samples: plan.samples, cursor: 0, name: instruction.name, line: instruction.line };
          motionPending = true;
          motion.moveJ(moving.samples[0].angles);
          state.message = `执行第 ${instruction.line} 行：MOVL ${instruction.name}（插补点 1/${moving.samples.length}）`;
          return;
        }
        case 'GRIP':
          grip(instruction.value);
          motionPending = false;
          state.message = `执行第 ${instruction.line} 行：GRIP ${instruction.value}（${instruction.value ? '夹紧' : '松开'}）${position()}`;
          return;
        case 'WAIT':
          waiting = { until: clock() + instruction.seconds * 1000 };
          motionPending = false;
          state.message = `执行第 ${instruction.line} 行：等待 ${instruction.seconds} 秒${position()}`;
          return;
        case 'SPEED':
          if (savedSpeed === null) savedSpeed = motion.state.speed;
          motion.state.speed = Math.min(Math.max(instruction.percent, 10), 100);
          state.speedPercent = motion.state.speed;
          motionPending = false;
          state.message = `执行第 ${instruction.line} 行：速度倍率 ${motion.state.speed}%${position()}`;
          return;
        default:
          motionPending = false;
          return;
      }
    } catch (error) {
      release(DSL_STATUS.IDLE, `第 ${instruction.line} 行执行失败：${error.message}`);
    }
  };

  return {
    state,
    /** 供界面预览：不执行，只返回解析结果。 */
    parse(code, poses) { return parseDSLScript(code, poses); },
    start(code, poses, options = {}) {
      if (state.running) throw new Error('程序正在执行');
      if (!motion.allowed()) throw new Error('请先在左侧允许机械臂运动');
      if (!api.robot?.visible) throw new Error('请先切换到 GLB 外观模型');
      const parsed = parseDSLScript(code, poses);
      if (parsed.errors.length) throw new Error(parsed.errors[0].message);
      if (!parsed.instructions.length) throw new Error('程序为空，请先写入至少一条指令');
      savedPoseOfAngles = options.poseOfAngles ?? null;

      motion.stop();
      stopVersion = motion.state.stopVersion;
      state.instructions = parsed.instructions;
      state.index = -1;
      state.progress = 0;
      state.lastMOVL = null;
      state.speedPercent = motion.state.speed;
      savedSpeed = null;
      motionPending = false; moving = null; waiting = null;
      setStatus(DSL_STATUS.RUNNING);
      state.message = `开始执行，共 ${parsed.instructions.length} 条指令`;
      motion.state.programActive = true;
      next();
    },
    pause() {
      if (state.status !== DSL_STATUS.RUNNING) return false;
      setStatus(DSL_STATUS.PAUSED);
      state.message = '程序已暂停，机械臂在完成当前小段后停在原地；点「继续」恢复';
      return true;
    },
    resume() {
      if (state.status !== DSL_STATUS.PAUSED) return false;
      setStatus(DSL_STATUS.RUNNING);
      state.message = '继续执行';
      return true;
    },
    stop() {
      motion.stop();
      release(DSL_STATUS.IDLE, '程序已停止，机械臂保持当前位置');
    },
    update() {
      if (!state.running) return;
      if (state.status === DSL_STATUS.PAUSED) return;
      if (!motion.allowed()) { release(DSL_STATUS.ESTOP, '急停或未使能：程序已中断，机械臂保持当前位置'); return; }
      if (motion.state.stopVersion !== stopVersion) {
        release(DSL_STATUS.IDLE, '程序已中断，保持当前位置；需重新点击执行');
        return;
      }
      try {
        if (moving) {
          if (motion.busy()) return;
          moving.cursor += 1;
          if (moving.cursor >= moving.samples.length) {
            moving = null; motionPending = false;
            next();
            return;
          }
          motion.moveJ(moving.samples[moving.cursor].angles);
          state.message = `执行第 ${moving.line} 行：MOVL ${moving.name}（插补点 ${moving.cursor + 1}/${moving.samples.length}）`;
          return;
        }
        if (waiting) {
          if (clock() < waiting.until) return;
          waiting = null;
          next();
          return;
        }
        if (motionPending) {
          if (motion.busy()) return;
          motionPending = false;
        }
        next();
      } catch (error) {
        release(DSL_STATUS.IDLE, `程序中断：${error.message}`);
      }
    },
  };
}
