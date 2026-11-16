import * as THREE from 'three';
import { threePointFrame } from './calibration-math.js';
import { createTCPOffset, worldToUserFrame, fitTCPFromPoses } from './Calibration.js';

const byId = (id) => document.getElementById(id);
const triple = (text) => {
  const values = text.trim().split(/[,，\s]+/).map(Number);
  if (values.length !== 3 || !values.every(Number.isFinite)) throw new Error('请输入三个有限数，例如 1,2,3');
  return values;
};
const stamp = () => new Date().toISOString();
const vectorText = (values) => values.map((v) => Number(v).toFixed(4)).join(', ');
const storageKey = 'robot-arm-calibration-experiment2-v3';
const sourceLabel = (source) => ({
  'asset-nominal': '模型预设尖端（尚未标定）',
  'manual-or-mixed-points': '手动输入或混合采样',
  'known-tool-offset': '已知工具偏移',
  'configured-tcp-capture': '已知TCP仿真采样',
  'workpiece-markers': '工件几何标记（仿真坐标）',
  'four-point-sphere': '四点法 TCP 标定',
  'four-point-sphere-demo': '四点法算法示例（非实测）',
})[source] || source;

export function bindCalibration(api, motion, scene) {
  const flange = api.robot.getObjectByName('link_flange');
  if (!flange) throw new Error('模型缺少 link_flange，无法标定工具');
  api.robot.updateMatrixWorld(true);
  const tool = new THREE.Object3D(); tool.name = 'calibrated-tool';
  const nominal = flange.matrixWorld.clone().invert().multiply(api.tcp.matrixWorld);
  nominal.decompose(tool.position, tool.quaternion, tool.scale);
  byId('tcp-translation').value = tool.position.toArray().join(',');
  const initialRpy = new THREE.Euler().setFromQuaternion(tool.quaternion, 'ZYX');
  byId('tcp-rotation').value = [initialRpy.x, initialRpy.y, initialRpy.z].map(THREE.MathUtils.radToDeg).join(',');
  flange.add(tool);
  api.assetTcp = api.tcp;
  api.tcp = tool; // 后续位姿显示、示教和轨迹统一使用当前工具；原资产参考点另行保留。
  tool.add(new THREE.AxesHelper(0.8));
  tool.add(new THREE.Mesh(new THREE.SphereGeometry(.12, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffd06b })));
  const frameAxes = new THREE.AxesHelper(2); frameAxes.matrixAutoUpdate = false; frameAxes.visible = false; scene.add(frameAxes);
  let userFrame = null, frameSource = 'manual-world-input';
  let pointSources = {}, frameCaptures = {}, activeTool = { source: 'asset-nominal', matrix: nominal.toArray() };
  const validations = [];
  let tcpSamples = [], tcpFit = null;
  const stopped = () => { if (motion.busy() || motion.state?.programActive) throw new Error('请先停止运动和程序再采样或应用标定'); };
  const tolerance = () => {
    const value = Number(byId('tcp-tolerance').value);
    if (!Number.isFinite(value) || value <= 0) throw new Error('接触误差容差必须为正有限数');
    return value;
  };
  const record = () => ({ version: 3, model: 'robot_articulated.glb', unit: 'asset-unit', timestamp: stamp(), activeTool,
    tcpCalibration: { samples: structuredClone(tcpSamples), fit: tcpFit && structuredClone(tcpFit), tolerance: Number(byId('tcp-tolerance').value) },
    toolInputs: { translation: byId('tcp-translation').value, rotationDeg: byId('tcp-rotation').value },
    frameInputs: ['o', 'x', 'xy'].map((key) => byId(`frame-${key}`).value), frameSource, pointSources, frameCaptures, userFrame, validations });
  const setText = (id, text) => { const el = byId(id); if (el) el.textContent = text; };
  const save = () => {
    try { localStorage.setItem(storageKey, JSON.stringify(record())); setText('record-status', `已保存至浏览器本地 · ${new Date().toLocaleTimeString('zh-CN')}`); }
    catch { setText('record-status', '浏览器本地保存不可用，请导出 JSON 保存记录'); }
  };
  const act = (id, output, fn) => byId(id)?.addEventListener('click', () => {
    try { fn(); } catch (error) { setText(output, error.message); }
  });
  const applyTool = (offset, rpyDeg, source) => {
    stopped();
    if (api.workstation?.attached) throw new Error('请先释放工件再更改 TCP');
    const matrix = createTCPOffset(offset, rpyDeg.map(THREE.MathUtils.degToRad));
    matrix.decompose(tool.position, tool.quaternion, tool.scale);
    activeTool = { offset, rpyDeg, matrix: matrix.toArray(), source, timestamp: stamp() };
    byId('tcp-translation').value = offset.join(',');
    userFrame = null; frameAxes.visible = false; frameCaptures = {}; pointSources = {};
    ['o', 'x', 'xy'].forEach((key) => { byId(`frame-${key}`).value = ''; });
    validations.length = 0;
    setText('frame-matrix', '尚无结果'); setText('frame-validation', '工具已改变，请重新验证');
    byId('frame-result').textContent = '工具偏移已设置，可记录工件 O、X、XY 三点。';
    byId('known-tcp-result').textContent = `已应用${source === 'known-tool-offset' ? '已知 TCP 偏移' : sourceLabel(source)}\n平移：${vectorText(offset)}\nRPY(度)：${vectorText(rpyDeg)}\n工具→法兰矩阵：\n${JSON.stringify(matrix.toArray())}`;
    api.workstation?.updateGripper();
    save();
  };
  act('apply-known-tcp', 'known-tcp-result', () => {
    applyTool(triple(byId('tcp-translation').value), triple(byId('tcp-rotation').value), 'known-tool-offset');
  });
  const renderSamples = () => {
    const list = byId('tcp-sample-list'); list.replaceChildren();
    tcpSamples.forEach((sample, index) => {
      const row = document.createElement('tr');
      for (const value of [`P${index + 1}`, vectorText(sample.matrix.slice(12, 15)), sample.source === 'synthetic-demo' ? '算法示例' : '接触采样']) {
        const cell = document.createElement('td'); cell.textContent = value; row.appendChild(cell);
      }
      list.appendChild(row);
    });
    setText('tcp-sample-count', `已采集 ${tcpSamples.length} 个位姿 / 至少 4 个`);
    byId('tcp-fit').disabled = tcpSamples.length < 4;
    byId('tcp-apply-fit').disabled = !tcpFit;
  };
  const validateSamples = (samples) => {
    if (!Array.isArray(samples) || samples.length > 100) throw new Error('TCP 样本必须为数组，最多 100 组');
    const clean = samples.map((sample) => {
      const matrix = sample?.matrix;
      if (!Array.isArray(matrix) || matrix.length !== 16 || !matrix.every(Number.isFinite)) throw new Error('法兰位姿必须为 16 个有限数的列主序矩阵');
      const transform = new THREE.Matrix4().fromArray(matrix), e = transform.elements;
      const x = new THREE.Vector3().setFromMatrixColumn(transform, 0), y = new THREE.Vector3().setFromMatrixColumn(transform, 1), z = new THREE.Vector3().setFromMatrixColumn(transform, 2);
      const scale = x.length();
      if (Math.abs(e[3]) + Math.abs(e[7]) + Math.abs(e[11]) + Math.abs(e[15] - 1) > 1e-8 || scale < 1e-12 || transform.determinant() <= 0 ||
        Math.abs(y.length() / scale - 1) > 1e-6 || Math.abs(z.length() / scale - 1) > 1e-6 ||
        [x.dot(y), y.dot(z), z.dot(x)].some((dot) => Math.abs(dot) > 1e-6 * scale * scale)) throw new Error('法兰矩阵必须为右手刚体位姿或统一缩放位姿');
      if (sample.angles != null && (!Array.isArray(sample.angles) || sample.angles.length !== api.joints.length || !sample.angles.every(Number.isFinite))) throw new Error('采样关节角格式错误');
      return { matrix: [...matrix], source: sample.source === 'synthetic-demo' ? 'synthetic-demo' : 'flange-contact',
        timestamp: typeof sample.timestamp === 'string' ? sample.timestamp : stamp(), angles: sample.angles ? [...sample.angles] : null };
    });
    if (clean.some((sample) => sample.source !== clean[0].source)) throw new Error('算法示例与接触采样不能混用，请先清空采样');
    return clean;
  };
  const setTCPSamples = (samples) => {
    stopped(); const clean = validateSamples(samples);
    tcpSamples = clean; tcpFit = null; renderSamples();
    setText('tcp-fit-result', '样本已更新，请重新拟合。'); save();
  };
  const captureTCPSample = () => {
    stopped(); flange.updateWorldMatrix(true, false);
    if (tcpSamples.some((sample) => sample.source === 'synthetic-demo')) throw new Error('请先清空算法示例，再采集当前法兰位姿');
    const matrix = flange.matrixWorld.toArray();
    if (tcpSamples.some((sample) => sample.matrix.every((value, i) => Math.abs(value - matrix[i]) < 1e-8))) throw new Error('该法兰位姿已采集，请改变姿态后重新接触固定点');
    setTCPSamples([...tcpSamples, { matrix, angles: api.joints.map((joint) => joint.angle), source: 'flange-contact', timestamp: stamp() }]);
  };
  const fitTCP = () => {
    stopped(); tcpFit = null; byId('tcp-apply-fit').disabled = true;
    const limit = tolerance();
    const fit = fitTCPFromPoses(tcpSamples.map((sample) => sample.matrix));
    tcpFit = { offset: fit.offset.toArray(), center: fit.center.toArray(), residuals: fit.residuals, rms: fit.rms, maxResidual: fit.maxResidual,
      sampleCount: fit.sampleCount, scale: fit.scale, localOffsets: fit.localOffsets.map((value) => value.toArray()),
      sphere: { ...fit.sphere, center: fit.sphere.center.toArray() }, source: tcpSamples[0].source, timestamp: stamp() };
    const passed = fit.maxResidual <= limit && fit.sphere.maxResidual <= limit;
    setText('tcp-fit-result', `${tcpFit.source === 'synthetic-demo' ? '算法示例（非实测）\n' : ''}法兰系 TCP 偏移：${vectorText(tcpFit.offset)}\n固定接触点（世界）：${vectorText(tcpFit.center)}\n球半径：${fit.sphere.radius.toFixed(6)}\n球面 RMS：${fit.sphere.rms.toExponential(3)}\n接触 RMS：${fit.rms.toExponential(3)}\n最大接触误差：${fit.maxResidual.toExponential(3)}\n采样条件指标：${fit.sphere.condition.toFixed(2)}\n${passed ? '误差在容差内，可应用。' : '误差超过容差，禁止应用；请检查是否接触同一固定点。'}`);
    byId('tcp-apply-fit').disabled = !passed;
    save(); return structuredClone(tcpFit);
  };
  const applyFittedTCP = () => {
    stopped(); if (!tcpFit) throw new Error('请先拟合 TCP');
    const limit = tolerance();
    if (tcpFit.maxResidual > limit || tcpFit.sphere.maxResidual > limit) throw new Error('拟合误差超过容差，不能应用');
    const source = tcpFit.source === 'synthetic-demo' ? 'four-point-sphere-demo' : 'four-point-sphere';
    applyTool(tcpFit.offset, triple(byId('tcp-rotation').value), source);
  };
  act('tcp-capture', 'tcp-fit-result', captureTCPSample);
  act('tcp-undo', 'tcp-fit-result', () => setTCPSamples(tcpSamples.slice(0, -1)));
  act('tcp-clear-samples', 'tcp-fit-result', () => setTCPSamples([]));
  act('tcp-fit', 'tcp-fit-result', fitTCP);
  act('tcp-apply-fit', 'tcp-fit-result', applyFittedTCP);
  byId('tcp-tolerance').addEventListener('input', () => {
    tcpFit = null; byId('tcp-apply-fit').disabled = true;
    try { tolerance(); setText('tcp-fit-result', '容差已改变，请重新拟合检查。'); save(); }
    catch (error) { setText('tcp-fit-result', error.message); }
  });
  act('tcp-demo', 'tcp-fit-result', () => {
    const offset = new THREE.Vector3(.2, .4, .8), center = new THREE.Vector3(2, 3, 4);
    const orientations = [[0, 0, 0], [1.3, .2, -.3], [-.4, 1.4, .2], [.3, -.5, 1.7]];
    setTCPSamples(orientations.map((rpy) => {
      const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(...rpy));
      const position = center.clone().sub(offset.clone().applyQuaternion(rotation));
      return { matrix: new THREE.Matrix4().compose(position, rotation, new THREE.Vector3(1, 1, 1)).toArray(), source: 'synthetic-demo', timestamp: stamp() };
    }));
    setText('tcp-fit-result', '已载入 4 个理想接触位姿，仅供算法验证；机械臂未移动。');
  });
  renderSamples();
  const clearFrameEvidence = () => {
    validations.length = 0;
    setText('frame-matrix', '尚无结果'); setText('frame-validation', '尚未验证');
  };
  const invalidateFrame = () => { userFrame = null; frameAxes.visible = false; clearFrameEvidence(); byId('frame-result').textContent = '点位已改变，请重新建立用户坐标系'; };
  for (const key of ['o', 'x', 'xy']) {
    byId(`frame-${key}`).addEventListener('input', () => { frameSource = 'manual-world-input'; pointSources[key] = frameSource; delete frameCaptures[key]; invalidateFrame(); save(); });
    act(`capture-${key}`, 'frame-result', () => {
      stopped(); if (activeTool.source === 'asset-nominal') throw new Error('请先标定并应用 TCP，再采集用户坐标点；也可手工输入已测点坐标');
      api.robot.updateMatrixWorld(true);
      byId(`frame-${key}`).value = tool.getWorldPosition(new THREE.Vector3()).toArray().join(',');
      frameCaptures[key] = { angles: api.joints.map((j) => j.angle), world: tool.getWorldPosition(new THREE.Vector3()).toArray(), tool: structuredClone(activeTool), timestamp: stamp() };
      frameSource = 'configured-tcp-capture';
      pointSources[key] = frameSource;
      invalidateFrame(); save();
    });
  }
  const solveFrame = () => {
    stopped();
    const points = ['o', 'x', 'xy'].map((key) => triple(byId(`frame-${key}`).value));
    const sources = Object.values(pointSources);
    frameSource = sources.length === 3 && sources.every((value) => value === 'workpiece-markers') ? 'workpiece-markers' : sources.length === 3 && sources.every((value) => value === 'configured-tcp-capture') ? 'configured-tcp-capture' : 'manual-or-mixed-points';
    userFrame = { ...threePointFrame(...points), points, source: frameSource, pointSources: { ...pointSources }, timestamp: stamp(), tool: structuredClone(activeTool) };
    clearFrameEvidence();
    frameAxes.matrix.fromArray(userFrame.matrix); frameAxes.matrixWorldNeedsUpdate = true; frameAxes.visible = true;
    byId('frame-result').textContent = `用户坐标系已建立\n数据来源：${sourceLabel(frameSource)}\n场景中已显示用户坐标轴\n正交误差：${userFrame.orthogonalityError.toExponential(3)}\n轴向约束误差：${userFrame.axisError.toExponential(3)}\n坐标往返误差：${userFrame.roundTripError.toExponential(3)}`;
    byId('frame-matrix').textContent = `世界←用户矩阵（列主序）：\n${JSON.stringify(userFrame.matrix)}\n旋转行列式：${userFrame.determinant.toFixed(8)}\n三点的用户坐标：\n${userFrame.localPoints.map(vectorText).join('\n')}`;
    save();
  };
  act('solve-frame', 'frame-result', solveFrame);
  act('verify-frame', 'frame-validation', () => {
    if (!userFrame) throw new Error('请先建立用户坐标系');
    const world = new THREE.Vector3().fromArray(triple(byId('verify-point').value));
    const local = worldToUserFrame(world, new THREE.Matrix4().fromArray(userFrame.inverse));
    const recovered = local.clone().applyMatrix4(new THREE.Matrix4().fromArray(userFrame.matrix));
    const result = { world: world.toArray(), user: local.toArray(), recovered: recovered.toArray(), error: world.distanceTo(recovered), timestamp: stamp() };
    validations.push(result);
    byId('frame-validation').textContent = `转换到用户坐标：\n${vectorText(result.user)}\n还原到世界坐标：\n${vectorText(result.recovered)}\n往返误差：${result.error.toExponential(3)}`;
    save();
  });
  act('export-calibration', 'record-status', () => {
    tolerance();
    const url = URL.createObjectURL(new Blob([JSON.stringify(record(), null, 2)], { type: 'application/json' }));
    const link = document.createElement('a'); link.href = url; link.download = `robot-calibration-${stamp().replace(/[:.]/g, '-')}.json`;
    document.body.appendChild(link); link.click(); link.remove();
    byId('record-status').textContent = '标定 JSON 已生成，请在浏览器下载列表查看。';
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  const restore = (data) => {
    stopped();
    if (!data || ![1, 2, 3].includes(data.version) || data.model !== 'robot_articulated.glb') throw new Error('标定记录格式或模型不匹配');
    if (api.workstation?.attached) throw new Error('请先释放工件再导入标定');
    const samples = validateSamples(data.tcpCalibration?.samples ?? []);
    const limit = data.tcpCalibration?.tolerance ?? .01;
    if (!Number.isFinite(limit) || limit <= 0) throw new Error('接触误差容差必须为正有限数');
    const inputs = data.frameInputs ?? ['', '', ''];
    if (!Array.isArray(inputs) || inputs.length !== 3 || inputs.some((value) => typeof value !== 'string')) throw new Error('三点数据格式错误');
    inputs.forEach((text) => { if (text.trim()) triple(text); });
    const known = data.activeTool?.source === 'known-tool-offset' ? data.activeTool : null;
    const translation = data.toolInputs?.translation ?? (known ? known.offset.join(',') : byId('tcp-translation').value);
    const rotation = data.toolInputs?.rotationDeg ?? (known ? known.rpyDeg.join(',') : byId('tcp-rotation').value);
    triple(translation); triple(rotation);
    // 导入仅恢复输入，绝不沿用文件中的拟合结果或自动移动工具。
    nominal.decompose(tool.position, tool.quaternion, tool.scale);
    activeTool = { source: 'asset-nominal', matrix: nominal.toArray() };
    tcpSamples = samples; tcpFit = null; byId('tcp-tolerance').value = limit;
    renderSamples(); validations.length = 0;
    ['o', 'x', 'xy'].forEach((key, i) => { byId('frame-' + key).value = inputs[i]; });
    byId('tcp-translation').value = translation; byId('tcp-rotation').value = rotation;
    frameSource = data.frameSource || 'imported'; frameCaptures = data.frameCaptures || {};
    pointSources = data.pointSources || { o: frameSource, x: frameSource, xy: frameSource };
    userFrame = null; frameAxes.visible = false;
    setText('known-tcp-result', '已恢复参数，当前使用模型预设 TCP；请重新拟合并应用，或应用已知工具偏移。');
    setText('tcp-fit-result', '已恢复样本，未采用导入文件中的拟合结果，请重新拟合。');
    setText('frame-result', '已恢复三点输入，请应用 TCP 后重新建立用户坐标系。');
    setText('frame-matrix', '尚无结果'); setText('frame-validation', '尚未验证');
    setText('record-status', '已恢复记录，标定未自动应用。');
    api.workstation?.updateGripper();
  };
  byId('import-calibration')?.addEventListener('change', async (event) => {
    try {
      stopped(); const file = event.target.files[0]; if (!file) return;
      if (file.size > 2_000_000) throw new Error('标定文件不能超过2MB');
      const data = JSON.parse(await file.text()); restore(data); save();
    } catch (error) { setText('record-status', error.message); }
    event.target.value = '';
  });
  act('clear-calibration', 'record-status', () => {
    stopped(); if (api.workstation?.attached) throw new Error('请先释放工件再重置标定');
    userFrame = null; frameCaptures = {}; pointSources = {}; validations.length = 0;
    tcpSamples = []; tcpFit = null; renderSamples();
    nominal.decompose(tool.position, tool.quaternion, tool.scale); activeTool = { source: 'asset-nominal', matrix: nominal.toArray() };
    byId('tcp-translation').value = tool.position.toArray().join(',');
    byId('tcp-rotation').value = [initialRpy.x, initialRpy.y, initialRpy.z].map(THREE.MathUtils.radToDeg).join(',');
    byId('tcp-tolerance').value = .01;
    ['o', 'x', 'xy'].forEach((key) => { byId(`frame-${key}`).value = ''; });
    frameAxes.visible = false;
    setText('tcp-fit-result', '尚未拟合。'); setText('frame-matrix', '尚无结果');
    setText('known-tcp-result', '已恢复模型预设 TCP'); setText('frame-result', '尚未建立用户坐标系'); setText('frame-validation', '尚未验证');
    api.workstation?.updateGripper(); save();
  });
  try { const saved = localStorage.getItem(storageKey); if (saved) restore(JSON.parse(saved)); }
  catch { setText('record-status', '旧记录不可读取，请重新采样或导入'); }
  api.calibration = { tool, get record() { return structuredClone(record()); },
    setTCPSamples, captureTCPSample, fitTCP, applyFittedTCP,
    invalidateUserFrame(reason) {
      userFrame = null; frameAxes.visible = false; clearFrameEvidence();
      byId('frame-result').textContent = reason; save();
    },
    setUserPoints(points, source = 'manual-world-input') {
      stopped();
      const values = [points.O, points.Px, points.Pxy].map((point) => triple((point?.isVector3 ? point.toArray() : point).join(',')));
      ['o', 'x', 'xy'].forEach((key, i) => { byId(`frame-${key}`).value = values[i].join(','); pointSources[key] = source; });
      frameSource = source; frameCaptures = {}; invalidateFrame(); save();
    },
    get userMatrix() { return userFrame ? new THREE.Matrix4().fromArray(userFrame.inverse) : null; },
    get tcpOffsetMatrix() { tool.updateMatrix(); return tool.matrix.clone(); } };
  return { update() {
    api.robot.updateMatrixWorld(true);
    const worldMatrix = tool.matrixWorld.clone(), p = new THREE.Vector3(), q = new THREE.Quaternion(), scale = new THREE.Vector3();
    worldMatrix.decompose(p, q, scale);
    const euler = new THREE.Euler().setFromQuaternion(q, 'ZYX');
    const base = api.robot.getObjectByName('link_base');
    const basePosition = base ? p.clone().applyMatrix4(base.matrixWorld.clone().invert()) : p;
    byId('tcp-position').textContent = `X ${p.x.toFixed(3)}　Y ${p.y.toFixed(3)}　Z ${p.z.toFixed(3)}`;
    let text = `世界 xyz: ${vectorText(p.toArray())}\n基座 xyz: ${vectorText(basePosition.toArray())}\nRPY(°): ${vectorText([euler.x, euler.y, euler.z].map(THREE.MathUtils.radToDeg))}\n四元数 xyzw: ${vectorText(q.toArray())}\n工具来源: ${sourceLabel(activeTool.source)}`;
    if (userFrame) text += `\n用户 xyz: ${vectorText(p.clone().applyMatrix4(new THREE.Matrix4().fromArray(userFrame.inverse)).toArray())}`;
    byId('tcp-pose').textContent = text;
    frameAxes.visible = !!userFrame && api.robot.visible;
  } };
}
