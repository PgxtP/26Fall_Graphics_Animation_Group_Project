export function compileProgram(code, poses, joints) {
  const map = new Map();
  for (const pose of poses) {
    if (!/^P[1-9]\d*$/.test(pose.name) || map.has(pose.name)) throw new Error('位姿名称必须唯一，格式为 P1、P2…');
    if (!Array.isArray(pose.angles) || pose.angles.length !== joints.length || pose.angles.some((angle, i) => !Number.isFinite(angle) || angle < joints[i].lower - 1e-9 || angle > joints[i].upper + 1e-9)) throw new Error(`${pose.name} 关节角无效或超限`);
    map.set(pose.name, pose);
  }
  const instructions = [];
  code.split(/\r?\n/).forEach((raw, index) => {
    const line = raw.trim(); if (!line || line.startsWith('#')) return;
    const match = /^MOVJ\s+(P[1-9]\d*)$/i.exec(line);
    if (!match) throw new Error(`第 ${index + 1} 行格式错误，请使用 MOVJ P1`);
    const pose = map.get(match[1].toUpperCase());
    if (!pose) throw new Error(`第 ${index + 1} 行引用了未记录的 ${match[1]}`);
    instructions.push({ name: pose.name, angles: [...pose.angles], line: index + 1 });
  });
  const used = [...new Set(instructions.map((step) => step.name))];
  const distinct = new Set(used.map((name) => map.get(name).angles.map((v) => v.toFixed(8)).join(',')));
  if (used.length < 4 || distinct.size < 4) throw new Error('程序需引用至少4个不同的已记录位姿');
  return instructions;
}

export function createProgramRunner(api, motion) {
  const state = { running: false, index: -1, instructions: [], message: '尚未执行' };
  let stopVersion;
  const startSegment = () => {
    const step = state.instructions[state.index];
    motion.moveJ(step.angles);
    state.message = `执行第 ${step.line} 行：MOVJ ${step.name}（${state.index + 1}/${state.instructions.length}）`;
  };
  return { state,
    start(code, poses) {
      if (state.running) throw new Error('程序正在执行');
      if (!motion.allowed()) throw new Error('请先在左侧允许机械臂运动');
      if (!api.robot.visible) throw new Error('请先切换到 GLB 外观模型');
      const instructions = compileProgram(code, poses, api.joints);
      motion.stop(); stopVersion = motion.state.stopVersion;
      state.instructions = instructions; state.index = 0; state.running = true;
      motion.state.programActive = true; startSegment();
    },
    stop() { motion.stop(); state.running = false; state.message = '程序已停止，机械臂保持当前位置'; },
    update() {
      if (!state.running) return;
      if (!motion.allowed() || motion.state.stopVersion !== stopVersion) {
        state.running = false; motion.state.programActive = false; state.message = '程序已中断，保持当前位置；需重新点击执行'; return;
      }
      if (motion.busy()) return;
      state.index++;
      if (state.index >= state.instructions.length) {
        state.running = false; motion.state.programActive = false; state.message = `程序执行完成，共 ${state.instructions.length} 条 MOVJ`; return;
      }
      startSegment();
    },
  };
}

