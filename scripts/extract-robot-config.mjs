import { readFile, writeFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const bytes = await readFile(new URL('../public/models/robot3/robot_articulated.glb', import.meta.url));
const map = JSON.parse(await readFile(new URL('../public/models/robot3/joint_map.json', import.meta.url), 'utf8'));
const gltf = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
gltf.scene.traverse(node => node.quaternion.normalize());
const placement = new THREE.Group();
placement.rotation.x = Math.PI; // 将倒置的导出基座转为 Y 向上；保留 glTF 原始米制尺寸。
placement.add(gltf.scene);
placement.updateMatrixWorld(true);
const bounds = new THREE.Box3().setFromObject(gltf.scene);
const pivot = gltf.scene.getObjectByName('J1').getWorldPosition(new THREE.Vector3());
placement.position.set(-pivot.x, -bounds.min.y, -pivot.z);
placement.updateMatrixWorld(true);
const chain = [];
let node = gltf.scene.getObjectByName(map.flange.name);
while (node !== placement) {
  if (!node) throw new Error('法兰节点不在模型根节点之下');
  const jointIndex = map.joints.findIndex(j => j.name === node.name);
  chain.unshift({
    name: node.name, translation: node.position.toArray(),
    quaternion: node.quaternion.toArray(), scale: node.scale.toArray(), jointIndex,
    ...(jointIndex >= 0 ? { axis: map.joints[jointIndex].motion_axis } : {}),
  });
  node = node.parent;
}
const config = {
  source: 'public/models/robot3/robot_articulated.glb',
  sha256: createHash('sha256').update(bytes).digest('hex'),
  unitNote: '按 glTF 约定 1 资产单位 = 1 m；未做真机尺寸标定，不代表实物尺寸',
  axisConvention: '父坐标系中的轴；局部变换 T(t) R_axis(theta) R_zero S',
  placement: { position: placement.position.toArray(), quaternion: placement.quaternion.toArray(), scale: placement.scale.toArray() },
  baseMatrix: placement.matrixWorld.toArray(),
  jointLimits: map.joints.map(j => ({ min: THREE.MathUtils.degToRad(j.limit.lower), max: THREE.MathUtils.degToRad(j.limit.upper) })),
  chain,
};
await writeFile(new URL('../src/robot/robot3Config.js', import.meta.url),
  '// 由 npm run robot:extract 从课程 GLB 提取；请勿手改矩阵。\nexport const ROBOT_CONFIG = ' + JSON.stringify(config, null, 2) + ';\n');
console.log(JSON.stringify({ sha256: config.sha256, placement: config.placement, chain: chain.map(n => n.name) }, null, 2));
