import * as THREE from 'three';

const vector = (text, fallback) => {
  const values = text == null ? fallback : text.trim().split(/\s+/).map(Number);
  if (values.length !== 3 || !values.every(Number.isFinite)) throw new Error('URDF 三维向量格式错误');
  return values;
};

export function parseUrdf(xml) {
  if (xml.querySelector('parsererror')) throw new Error('URDF XML 解析失败');
  const links = [...xml.querySelectorAll('robot > link')].map((link) => link.getAttribute('name'));
  const joints = [...xml.querySelectorAll('robot > joint')].map((joint) => {
    const origin = joint.querySelector('origin');
    const limit = joint.querySelector('limit');
    const number = (key) => limit?.hasAttribute(key) ? Number(limit.getAttribute(key)) : null;
    return {
      name: joint.getAttribute('name'), type: joint.getAttribute('type'),
      parent: joint.querySelector('parent')?.getAttribute('link'),
      child: joint.querySelector('child')?.getAttribute('link'),
      xyz: vector(origin?.getAttribute('xyz'), [0, 0, 0]),
      rpy: vector(origin?.getAttribute('rpy'), [0, 0, 0]),
      axis: vector(joint.querySelector('axis')?.getAttribute('xyz'), [1, 0, 0]),
      lower: number('lower'), upper: number('upper'), effort: number('effort'), velocity: number('velocity'),
    };
  });
  return { links, joints };
}

export function setJointValue(joint, value) {
  if (!Number.isFinite(value)) throw new Error('关节值必须是有限数');
  const angle = joint.type === 'fixed' ? 0 : joint.type === 'continuous'
    ? value : THREE.MathUtils.clamp(value, joint.lower, joint.upper);
  if (joint.type === 'prismatic') {
    const parentAxis = joint.axis.clone().applyQuaternion(joint.restQuaternion);
    joint.node.position.copy(joint.restPosition).addScaledVector(parentAxis, angle);
  } else {
    joint.node.quaternion.copy(joint.restQuaternion)
      .multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, angle));
  }
  joint.value = angle;
  joint.angle = angle;
  return angle;
}

export function buildUrdfModel(definition) {
  const warnings = [];
  const links = new Map();
  for (const name of definition.links) {
    if (!name || links.has(name)) throw new Error('URDF link 名称缺失或重复');
    const link = new THREE.Object3D(); link.name = name; links.set(name, link);
  }
  // 教学容错：仅为本资产漏声明的 tcp 创建内存占位，源文件保持原样。
  for (const joint of definition.joints) {
    if (joint.child === 'tcp' && !links.has('tcp')) {
      const tcp = new THREE.Object3D(); tcp.name = 'tcp'; links.set('tcp', tcp);
      warnings.push('URDF 缺少 tcp link 声明：仅在内存创建占位节点。');
    }
  }
  const children = new Set();
  const names = new Set();
  for (const joint of definition.joints) {
    if (!joint.name || names.has(joint.name)) throw new Error('URDF joint 名称缺失或重复');
    names.add(joint.name);
    if (!links.has(joint.parent) || !links.has(joint.child)) throw new Error(`${joint.name} 引用了未声明的 link`);
    if (children.has(joint.child)) throw new Error('URDF 子连杆存在多个父关节');
    children.add(joint.child);
  }
  const roots = [...links.keys()].filter((name) => !children.has(name));
  if (roots.length !== 1) throw new Error('URDF 必须只有一个根连杆');
  const visited = new Set();
  const visit = (name) => {
    if (visited.has(name)) throw new Error('URDF 存在闭环');
    visited.add(name);
    definition.joints.filter((joint) => joint.parent === name).forEach((joint) => visit(joint.child));
  };
  visit(roots[0]);
  if (visited.size !== links.size) throw new Error('URDF 存在断开的连杆或闭环');

  const root = new THREE.Group(); root.name = 'URDF 骨架（无可视网格）';
  root.rotation.x = -Math.PI / 2; // ROS Z-up → three.js Y-up，只在最外层转换。
  root.add(links.get(roots[0]));
  const joints = definition.joints.map((entry) => {
    if (!['fixed', 'revolute', 'continuous', 'prismatic'].includes(entry.type)) throw new Error(`暂不支持关节类型 ${entry.type}`);
    if (['revolute', 'prismatic'].includes(entry.type) &&
      (entry.lower === null || entry.upper === null || !Number.isFinite(entry.lower) || !Number.isFinite(entry.upper) || entry.lower > entry.upper)) {
      throw new Error(`${entry.name} 缺少有效限位`);
    }
    const node = new THREE.Object3D(); node.name = entry.name;
    node.position.fromArray(entry.xyz);
    // URDF 固定轴 RPY：Rz(yaw) * Ry(pitch) * Rx(roll)。
    node.quaternion.setFromEuler(new THREE.Euler(...entry.rpy, 'ZYX'));
    const axis = new THREE.Vector3().fromArray(entry.axis);
    if (entry.type !== 'fixed' && axis.lengthSq() < 1e-12) throw new Error(`${entry.name} 的 axis 为零`);
    axis.normalize();
    links.get(entry.parent).add(node);
    node.add(links.get(entry.child));
    node.add(new THREE.AxesHelper(0.7));
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(), new THREE.Vector3().fromArray(entry.xyz),
    ]), new THREE.LineBasicMaterial({ color: 0x65d9c5 }));
    links.get(entry.parent).add(line);
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffc56a }));
    node.add(marker);
    return { ...entry, node, axis, restQuaternion: node.quaternion.clone(), restPosition: node.position.clone(), value: 0 };
  });
  root.updateMatrixWorld(true);
  return { root, joints, warnings };
}
