import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { setJointValue } from './urdf-model.js';

// 本资产由 Blender Z-up 转到 glTF Y-up，局部轴同样需要转换。
function gltfAxis(axis) {
  const source = typeof axis === 'string'
    ? { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] }[axis.toLowerCase()]
    : axis;
  if (!Array.isArray(source) || source.length !== 3 || !source.every(Number.isFinite)) throw new Error('关节轴格式无效');
  const result = new THREE.Vector3(source[0], source[2], -source[1]);
  if (result.lengthSq() < 1e-12) throw new Error('关节轴不能为零');
  return result.normalize();
}

export function collectRobot(robot, jointMap) {
  const expected = ['J1', 'J2', 'J3', 'J4', 'J5'];
  if (!Array.isArray(jointMap.joints) || jointMap.joints.length !== expected.length ||
    jointMap.joints.some((entry, index) => entry.name !== expected[index])) {
    throw new Error('JSON 关节数量或顺序错误，应为 J1–J5');
  }
  const entries = new Map(jointMap.joints.map((entry) => [entry.name, entry]));
  const joints = [];
  const tcpNodes = [];
  const warnings = [];
  robot.traverse((node) => {
    if (node.name === 'tcp') tcpNodes.push(node);
    if (!entries.has(node.name) && node.userData.joint !== true && !/^J\d+$/.test(node.name)) return;
    const entry = entries.get(node.name);
    if (!entry) throw new Error(`GLB 中存在未配置的关节 ${node.name}`);
    const lowerDeg = Number(entry.limit?.lower), upperDeg = Number(entry.limit?.upper);
    if (entry.unit !== 'deg' || !Number.isFinite(lowerDeg) || !Number.isFinite(upperDeg) || lowerDeg > upperDeg || lowerDeg > 0 || upperDeg < 0) {
      throw new Error(`${entry.name} 限位无效或不包含导出零位`);
    }
    const axis = gltfAxis(entry.axis);
    const metadata = node.userData;
    if (metadata.joint === true) {
      // GLB 如提供元数据，则要求其限位与 JSON 一致（默认单位为度）。
      const factor = metadata.limit?.unit === 'rad' ? 180 / Math.PI : 1;
      if (!metadata.limit || !Number.isFinite(metadata.limit.lower) || !Number.isFinite(metadata.limit.upper) ||
        Math.abs(metadata.limit.lower * factor - lowerDeg) > 1e-4 || Math.abs(metadata.limit.upper * factor - upperDeg) > 1e-4) {
        throw new Error(`${node.name} GLB 元数据限位与 JSON 不一致`);
      }
      if (metadata.axis == null || gltfAxis(metadata.axis).distanceTo(axis) > 1e-5) throw new Error(`${node.name} GLB 元数据轴向与 JSON 不一致`);
    } else {
      metadata.joint = true;
      metadata.axis = entry.axis;
      metadata.limit = { lower: lowerDeg, upper: upperDeg, unit: 'deg' };
      metadata.metadataSource = 'joint_map.json（运行时补充）';
      warnings.push(`${node.name}：GLB 未携带 joint 元数据，已从 JSON 补充到内存。`);
    }
    metadata.qZero = node.quaternion.clone();
    joints.push({ name: node.name, node, axis, type: 'revolute', lowerDeg, upperDeg,
      lower: THREE.MathUtils.degToRad(lowerDeg), upper: THREE.MathUtils.degToRad(upperDeg),
      angle: 0, value: 0, restQuaternion: metadata.qZero.clone() });
  });
  if (joints.length !== expected.length || joints.some((joint, index) => joint.name !== expected[index])) {
    throw new Error('GLB 关节数量、遍历顺序或名称与 JSON 不一致');
  }
  if (tcpNodes.length !== 1) throw new Error('GLB 必须包含唯一的 tcp 节点');
  for (let index = 1; index < joints.length; index++) {
    let parent = joints[index].node.parent;
    while (parent && parent !== joints[index - 1].node) parent = parent.parent;
    if (!parent) throw new Error(`${joints[index].name} 不在前一关节的下游`);
  }
  let ancestor = tcpNodes[0].parent;
  while (ancestor && ancestor !== joints.at(-1).node) ancestor = ancestor.parent;
  if (!ancestor) throw new Error('tcp 不在 J5 的下游');
  return { robot, joints, tcp: tcpNodes[0], warnings };
}

export async function loadRobot(url, jointMap) {
  const robot = (await new GLTFLoader().loadAsync(url)).scene;
  const result = collectRobot(robot, jointMap);
  return { ...result, setJoint(name, angleRad) {
    const joint = result.joints.find((entry) => entry.name === name);
    if (!joint) throw new Error(`未知关节 ${name}`);
    const value = setJointValue(joint, angleRad);
    robot.updateMatrixWorld(true);
    return value;
  } };
}
