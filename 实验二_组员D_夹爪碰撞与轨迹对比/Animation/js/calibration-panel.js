import * as THREE from 'three';
import { threePointFrame } from './calibration-math.js';
import { createTCPOffset, worldToUserFrame } from './Calibration.js';

const byId = (id) => document.getElementById(id);
const triple = (text) => {
  const values = text.trim().split(/[,，\s]+/).map(Number);
  if (values.length !== 3 || !values.every(Number.isFinite)) throw new Error('请输入三个有限数，例如 1,2,3');
  return values;
};
const stamp = () => new Date().toISOString();
const vectorText = (values) => values.map((v) => Number(v).toFixed(4)).join(', ');
const storageKey = 'robot-arm-calibration-v1';
const sourceLabel = (source) => ({
  'asset-nominal': '模型预设尖端（尚未标定）',
  'manual-or-mixed-points': '手动输入或混合采样',
  'known-tool-offset': '已知工具偏移',
  'configured-tcp-capture': '已知TCP仿真采样',
  'workpiece-markers': '工件几何标记（仿真坐标）',
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
  const stopped = () => { if (motion.busy()) throw new Error('请先停止运动再采样或应用标定'); };
  const record = () => ({ version: 2, model: 'robot_articulated.glb', unit: 'asset-unit', timestamp: stamp(), activeTool,
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
  act('apply-known-tcp', 'known-tcp-result', () => {
    stopped();
    const offset = triple(byId('tcp-translation').value), rpyDeg = triple(byId('tcp-rotation').value);
    const matrix = createTCPOffset(offset, rpyDeg.map(THREE.MathUtils.degToRad));
    matrix.decompose(tool.position, tool.quaternion, tool.scale);
    activeTool = { offset, rpyDeg, matrix: matrix.toArray(), source: 'known-tool-offset', timestamp: stamp() };
    userFrame = null; frameAxes.visible = false; frameCaptures = {}; pointSources = {};
    ['o', 'x', 'xy'].forEach((key) => { byId(`frame-${key}`).value = ''; });
    byId('frame-result').textContent = '工具偏移已设置，可记录工件 O、X、XY 三点。';
    byId('known-tcp-result').textContent = `已应用已知 TCP 偏移\n平移：${vectorText(offset)}\nRPY(度)：${vectorText(rpyDeg)}\n工具→法兰矩阵：\n${JSON.stringify(matrix.toArray())}`;
    save();
  });
  const invalidateFrame = () => { userFrame = null; frameAxes.visible = false; byId('frame-result').textContent = '点位已改变，请重新建立用户坐标系'; };
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
    const url = URL.createObjectURL(new Blob([JSON.stringify(record(), null, 2)], { type: 'application/json' }));
    const link = document.createElement('a'); link.href = url; link.download = `robot-calibration-${stamp().replace(/[:.]/g, '-')}.json`;
    document.body.appendChild(link); link.click(); link.remove();
    byId('record-status').textContent = '标定 JSON 已生成，请在浏览器下载列表查看。';
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  const restore = (data) => {
    if (![1, 2].includes(data.version) || data.model !== 'robot_articulated.glb') throw new Error('标定记录格式或模型不匹配');
    const inputs = data.frameInputs ?? ['', '', ''];
    if (!Array.isArray(inputs) || inputs.length !== 3 || inputs.some((value) => typeof value !== 'string')) throw new Error('三点数据格式错误');
    inputs.forEach((text) => { if (text.trim()) triple(text); });
    const known = data.activeTool?.source === 'known-tool-offset' ? data.activeTool : null;
    const translation = data.toolInputs?.translation ?? (known ? known.offset.join(',') : byId('tcp-translation').value);
    const rotation = data.toolInputs?.rotationDeg ?? (known ? known.rpyDeg.join(',') : byId('tcp-rotation').value);
    triple(translation); triple(rotation);
    ['o', 'x', 'xy'].forEach((key, i) => { byId('frame-' + key).value = inputs[i]; });
    byId('tcp-translation').value = translation; byId('tcp-rotation').value = rotation;
    frameSource = data.frameSource || 'imported'; frameCaptures = data.frameCaptures || {};
    pointSources = data.pointSources || { o: frameSource, x: frameSource, xy: frameSource };
    userFrame = null; frameAxes.visible = false;
    setText('known-tcp-result', '已恢复参数；请应用已知 TCP 偏移，再重新建立用户坐标系。');
    setText('record-status', '已恢复已知工具参数与三点记录，未自动应用。旧版枢轴样本不再参与本模块。');
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
    stopped(); userFrame = null; frameCaptures = {}; pointSources = {}; validations.length = 0;
    nominal.decompose(tool.position, tool.quaternion, tool.scale); activeTool = { source: 'asset-nominal', matrix: nominal.toArray() };
    ['o', 'x', 'xy'].forEach((key) => { byId(`frame-${key}`).value = ''; });
    frameAxes.visible = false;
    setText('known-tcp-result', '已恢复模型预设 TCP'); setText('frame-result', '尚未建立用户坐标系'); setText('frame-validation', '尚未验证'); save();
  });
  try { const saved = localStorage.getItem(storageKey); if (saved) restore(JSON.parse(saved)); }
  catch { setText('record-status', '旧记录不可读取，请重新采样或导入'); }
  api.calibration = { tool, get record() { return record(); },
    invalidateUserFrame(reason) {
      userFrame = null; frameAxes.visible = false;
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
