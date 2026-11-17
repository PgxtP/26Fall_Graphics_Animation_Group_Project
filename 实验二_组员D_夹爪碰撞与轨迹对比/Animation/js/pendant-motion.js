import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';

/**
 * 【王静 · 组员 D】夹爪碰撞检测
 *
 * 本文件在实验二底座的运动层上补一层「末端夹爪」几何检测：
 *   · 底座原有 isColliding() 只检查 link_1~link_5、工位台面、基座和 J3–J4 轴线；
 *     夹爪本体（掌部 + 左右指）以及夹爪已抓起的工件都不在检查范围内。
 *   · 新增 checkGripperCollision(gripperMesh, obstacles, options)，对夹爪子树（含已挂载
 *     工件）与任意障碍物集合做「三角面片相交 + 最小间距」检测，结果可被运动层直接使用。
 *   · 运动层把夹爪碰撞并入 isColliding()：一旦命中即回滚本帧位移、清空目标、置
 *     state.collision=true，并通过 state.collisionReason / collisionDetail 给出原因。
 *
 * 几何口径：网格三角面片（世界坐标）精确求交，间距用「顶点到三角面片」最近距离近似；
 * 不做连续碰撞检测（CCD），也不做物理响应；长度单位沿用资产单位。
 */

const COLLISION_DEFAULTS = Object.freeze({
  margin: 0,        // 安全间距：>0 时把「贴近但未穿透」也判为碰撞
  maxPairs: 20000,  // 单个障碍物最多参与检测的三角面对数，防止长帧
  includeHidden: false,
  probeDistance: 0.05, // 即使 margin=0，也统计该距离内的最近间距（超出则 minDistance=Infinity）
});

const triA = new THREE.Triangle();
const nA = new THREE.Vector3(), nB = new THREE.Vector3(), lineDir = new THREE.Vector3();
const pTmp = new THREE.Vector3(), qTmp = new THREE.Vector3();
const bigBox = new THREE.Box3(), otherBox = new THREE.Box3();
const scratchMatrix = new THREE.Matrix4();

/** 世界坐标三角面片缓存：同一网格的几何不变、世界矩阵不变时复用，避免逐帧重算。 */
const triangleCache = new WeakMap();

function worldMatrixKey(object) {
  return object.matrixWorld.elements.join(',');
}

function collectWorldTriangles(object, options) {
  const out = [];
  object.updateWorldMatrix(true, true);
  const meshes = [];
  object.traverse((node) => {
    if (!node.isMesh || !node.geometry?.attributes?.position) return;
    if (!options.includeHidden && node.visible === false) return;
    meshes.push(node);
  });
  for (const mesh of meshes) {
    const key = `${worldMatrixKey(mesh)}|${mesh.geometry.uuid}`;
    const hit = triangleCache.get(mesh);
    if (hit && hit.key === key) { out.push(...hit.triangles); continue; }
    scratchMatrix.copy(mesh.matrixWorld);
    const position = mesh.geometry.attributes.position;
    const index = mesh.geometry.index;
    const count = index?.count ?? position.count;
    const triangles = [];
    for (let i = 0; i + 2 < count; i += 3) {
      const i0 = index ? index.getX(i) : i;
      const i1 = index ? index.getX(i + 1) : i + 1;
      const i2 = index ? index.getX(i + 2) : i + 2;
      const a = new THREE.Vector3().fromBufferAttribute(position, i0).applyMatrix4(scratchMatrix);
      const b = new THREE.Vector3().fromBufferAttribute(position, i1).applyMatrix4(scratchMatrix);
      const c = new THREE.Vector3().fromBufferAttribute(position, i2).applyMatrix4(scratchMatrix);
      triangles.push([a, b, c]);
    }
    triangleCache.set(mesh, { key, triangles });
    out.push(...triangles);
  }
  return out;
}

function trianglesBox(triangles, target) {
  target.makeEmpty();
  for (const [a, b, c] of triangles) target.expandByPoint(a).expandByPoint(b).expandByPoint(c);
  return target;
}

function signedPlaneDistance(point, normal, pointOnPlane) {
  return normal.dot(pTmp.copy(point).sub(pointOnPlane));
}

/**
 * Möller 的三角面片相交判定：两三角形各自跨越对方所在平面，且交线段在公共直线上重叠。
 */
function triangleTriangleIntersect(a0, a1, a2, b0, b1, b2) {
  nB.crossVectors(qTmp.copy(b1).sub(b0), pTmp.copy(b2).sub(b0));
  if (nB.lengthSq() < 1e-24) return false; // 退化三角形
  const dA0 = signedPlaneDistance(a0, nB, b0);
  const dA1 = signedPlaneDistance(a1, nB, b0);
  const dA2 = signedPlaneDistance(a2, nB, b0);
  if ((dA0 > 0 && dA1 > 0 && dA2 > 0) || (dA0 < 0 && dA1 < 0 && dA2 < 0)) return false;

  nA.crossVectors(qTmp.copy(a1).sub(a0), pTmp.copy(a2).sub(a0));
  if (nA.lengthSq() < 1e-24) return false;
  const dB0 = signedPlaneDistance(b0, nA, a0);
  const dB1 = signedPlaneDistance(b1, nA, a0);
  const dB2 = signedPlaneDistance(b2, nA, a0);
  if ((dB0 > 0 && dB1 > 0 && dB2 > 0) || (dB0 < 0 && dB1 < 0 && dB2 < 0)) return false;

  lineDir.crossVectors(nA, nB);
  if (lineDir.lengthSq() < 1e-24) return false; // 共面：交给间距判定，不计穿透
  lineDir.normalize();

  const interval = (p0, p1, p2, d0, d1, d2) => {
    const project = (point) => lineDir.dot(point);
    let lo = Infinity, hi = -Infinity;
    const edge = (from, to, da, db) => {
      if ((da > 0 && db > 0) || (da < 0 && db < 0)) return;
      const t = da / (da - db);
      const value = project(from) + (project(to) - project(from)) * t;
      if (value < lo) lo = value;
      if (value > hi) hi = value;
    };
    edge(p0, p1, d0, d1); edge(p1, p2, d1, d2); edge(p2, p0, d2, d0);
    return [lo, hi];
  };
  const [aLo, aHi] = interval(a0, a1, a2, dA0, dA1, dA2);
  const [bLo, bHi] = interval(b0, b1, b2, dB0, dB1, dB2);
  if (!Number.isFinite(aLo) || !Number.isFinite(bLo)) return false;
  return aLo <= bHi + 1e-12 && bLo <= aHi + 1e-12;
}

/** 顶点集到三角面片集合的最小距离；提前退出阈值 limit，stride>1 时按步长抽样以限制长帧开销。 */
function minVertexTriangleDistance(vertices, triangles, limit, stride = 1) {
  let best = Infinity;
  for (let vi = 0; vi < vertices.length; vi += stride) {
    const vertex = vertices[vi];
    for (let ti = 0; ti < triangles.length; ti += stride) {
      const [a, b, c] = triangles[ti];
      triA.set(a, b, c);
      triA.closestPointToPoint(vertex, pTmp);
      const distance = pTmp.distanceTo(vertex);
      if (distance < best) {
        best = distance;
        if (best <= limit) return best;
      }
    }
  }
  return best;
}

function normalizeObstacles(obstacles) {
  const list = [];
  const push = (entry) => {
    if (!entry) return;
    if (entry.isObject3D) { list.push({ object: entry, name: entry.name || '未命名对象', margin: null }); return; }
    if (entry.object?.isObject3D) {
      list.push({
        object: entry.object,
        name: entry.name || entry.object.name || '未命名对象',
        margin: Number.isFinite(entry.margin) ? entry.margin : null,
      });
    }
  };
  if (Array.isArray(obstacles)) obstacles.forEach(push);
  else push(obstacles);
  return list;
}

function isPartOf(object, root) {
  for (let node = object; node; node = node.parent) if (node === root) return true;
  return false;
}

/**
 * 夹爪（含已挂载工件）与障碍物的碰撞检测。
 *
 * @param {THREE.Object3D|THREE.Object3D[]} gripperMesh 夹爪根节点，或节点数组（可把已抓工件一并传入）
 * @param {Array<THREE.Object3D|{object:THREE.Object3D,name?:string,margin?:number}>} obstacles 障碍物集合
 * @param {{margin?:number,maxPairs?:number,includeHidden?:boolean,probeDistance?:number}} [options]
 * @returns {{colliding:boolean,margin:number,hits:Array,minDistance:number,gripperTriangles:number,
 *            obstacleTriangles:number,obstaclesChecked:number,reason:string}}
 */
export function checkGripperCollision(gripperMesh, obstacles = [], options = {}) {
  // 显式传 undefined 的字段不能覆盖默认值，否则 maxPairs/probeDistance 会变成 NaN。
  const config = { ...COLLISION_DEFAULTS };
  for (const [key, value] of Object.entries(options ?? {})) if (value !== undefined) config[key] = value;
  const margin = Number.isFinite(config.margin) ? Math.max(0, config.margin) : 0;
  const sources = (Array.isArray(gripperMesh) ? gripperMesh : [gripperMesh]).filter((item) => item?.isObject3D);
  const result = {
    colliding: false, margin, hits: [], minDistance: Infinity,
    gripperTriangles: 0, obstacleTriangles: 0, obstaclesChecked: 0, reason: '',
  };
  if (!sources.length) { result.reason = '缺少夹爪节点'; return result; }

  const triangleOptions = { includeHidden: config.includeHidden === true };
  const gripperTriangles = [];
  for (const source of sources) gripperTriangles.push(...collectWorldTriangles(source, triangleOptions));
  result.gripperTriangles = gripperTriangles.length;
  if (!gripperTriangles.length) { result.reason = '夹爪没有可检测的三角网格'; return result; }

  const gripperVertices = [];
  for (const [a, b, c] of gripperTriangles) gripperVertices.push(a, b, c);

  const entries = normalizeObstacles(obstacles);
  const maxMargin = Math.max(margin, config.probeDistance, ...entries.map((entry) => entry.margin ?? margin));
  const gripperBounds = trianglesBox(gripperTriangles, new THREE.Box3());
  bigBox.copy(gripperBounds).expandByScalar(maxMargin);

  for (const entry of entries) {
    if (sources.some((source) => isPartOf(entry.object, source))) continue; // 障碍物就是夹爪自身
    if (sources.some((source) => isPartOf(source, entry.object))) continue; // 夹爪挂在障碍物内部
    result.obstaclesChecked += 1;
    const obstacleTriangles = collectWorldTriangles(entry.object, triangleOptions);
    result.obstacleTriangles += obstacleTriangles.length;
    if (!obstacleTriangles.length) continue;
    const obstacleMargin = entry.margin ?? margin;
    const obstacleBounds = trianglesBox(obstacleTriangles, new THREE.Box3());
    otherBox.copy(obstacleBounds).expandByScalar(maxMargin);
    if (!bigBox.intersectsBox(otherBox)) continue; // 粗筛：包围盒不相交

    // 0) 包含：整体位于对方内部时，三角面片不会相交，必须单独判定。
    if (obstacleBounds.containsBox(gripperBounds) || gripperBounds.containsBox(obstacleBounds)) {
      result.hits.push({
        gripper: sources.length === 1 ? sources[0].name || '夹爪' : '夹爪组',
        obstacle: entry.name, kind: 'penetration', distance: 0, containment: true,
      });
      result.minDistance = 0;
      result.colliding = true;
      continue;
    }

    // 1) 穿透：三角面片严格相交
    const stride = Math.max(1, Math.ceil((gripperTriangles.length * obstacleTriangles.length) / Math.max(1, config.maxPairs)));
    let penetrated = false;
    outer: for (let i = 0; i < gripperTriangles.length; i += stride) {
      const [a0, a1, a2] = gripperTriangles[i];
      for (let j = 0; j < obstacleTriangles.length; j += stride) {
        const [b0, b1, b2] = obstacleTriangles[j];
        if (triangleTriangleIntersect(a0, a1, a2, b0, b1, b2)) {
          penetrated = true;
          result.hits.push({
            gripper: sources.length === 1 ? sources[0].name || '夹爪' : '夹爪组',
            obstacle: entry.name, kind: 'penetration', distance: 0,
          });
          result.minDistance = 0;
          break outer;
        }
      }
    }
    if (penetrated) { result.colliding = true; continue; }

    // 2) 贴近：最小间距不超过安全间距
    const probe = Math.max(obstacleMargin, config.probeDistance);
    const obstacleVertices = [];
    for (const [a, b, c] of obstacleTriangles) obstacleVertices.push(a, b, c);
    const distanceStride = Math.max(1, Math.ceil(Math.sqrt((gripperVertices.length * obstacleTriangles.length) / Math.max(1, config.maxPairs))));
    const distance = Math.min(
      minVertexTriangleDistance(gripperVertices, obstacleTriangles, probe, distanceStride),
      minVertexTriangleDistance(obstacleVertices, gripperTriangles, probe, distanceStride),
    );
    if (distance < result.minDistance) result.minDistance = distance;
    if (obstacleMargin > 0 && distance <= obstacleMargin) {
      result.hits.push({
        gripper: sources.length === 1 ? sources[0].name || '夹爪' : '夹爪组',
        obstacle: entry.name, kind: 'clearance', distance, margin: obstacleMargin,
      });
      result.colliding = true;
    }
  }

  if (result.colliding) {
    const first = result.hits[0];
    result.reason = first.kind === 'penetration'
      ? `夹爪与「${first.obstacle}」发生穿透${first.containment ? '（整体位于障碍物内部）' : ''}`
      : `夹爪与「${first.obstacle}」间距 ${result.minDistance.toFixed(4)} 小于安全间距 ${first.margin}`;
  }
  return result;
}

/**
 * 从 api 里解析夹爪几何与障碍物集合：
 *   · 夹爪：优先 api.workstation.gripper（几何夹爪组），并带上已挂载工件；
 *     没有工位模块时退回 flange / tcp 子树，不依赖其他组员的文件也能工作。
 *   · 障碍物：工作台、基座、未挂载的工件，可由 options.extraObstacles 追加。
 */
export function resolveGripperScene(api, options = {}) {
  const station = api?.workstation;
  const robot = api?.robot;
  const parts = [];
  if (station?.gripper?.isObject3D) parts.push(station.gripper);
  if (station?.attached && station.workpiece?.isObject3D) parts.push(station.workpiece);
  if (!parts.length) {
    const fallback = robot?.getObjectByName?.('link_flange') ?? robot?.getObjectByName?.('tcp');
    if (fallback) parts.push(fallback);
  }
  const obstacles = [];
  if (station?.table?.isObject3D) obstacles.push({ object: station.table, name: '工作台' });
  const base = robot?.getObjectByName?.('link_base');
  if (base) obstacles.push({ object: base, name: '基座' });
  if (station?.workpiece?.isObject3D && !station.attached) obstacles.push({ object: station.workpiece, name: '工件' });
  for (const extra of options.extraObstacles ?? []) obstacles.push(extra);
  return { parts, obstacles };
}

export function createMotion(api, urdfModel, motionOptions = {}) {
  const state = {
    enabled: false, estop: false, speed: 25, jog: null, targets: new Map(), segment: null,
    programActive: false, stopVersion: 0, collision: false,
    collisionReason: '',        // 【D】最近一次碰撞的可读原因
    collisionDetail: null,      // 【D】最近一次碰撞的几何细节（命中对/间距/来源）
    collisionCount: 0,          // 【D】累计碰撞次数，供第 5 章统计
  };
  const stop = () => { state.jog = null; state.targets.clear(); state.segment = null; state.programActive = false; state.stopVersion++; };
  const allowed = () => state.enabled && !state.estop;
  const target = (name, radians) => {
    if (!allowed()) throw new Error('请先解除急停并打开关节使能');
    if (state.programActive) throw new Error('程序正在执行，请先停止程序再手动操作');
    if (!Number.isFinite(radians)) throw new Error('目标角度无效');
    const joint = api.joints.find((j) => j.name === name);
    if (!joint) throw new Error(`未知关节 ${name}`);
    const value = THREE.MathUtils.clamp(radians, joint.lower, joint.upper);
    state.jog = null; state.segment = null; state.targets.set(name, value); return value;
  };
  const apply = (joint, radians) => {
    const value = setJointValue(joint, radians), other = urdfModel.joints.find((j) => j.name === joint.name);
    if (other) setJointValue(other, value);
  };

  // --- 精细化分层安全检测（全连杆防触地 + 臂体自干涉 + J3-J4与末端链防撞） ---
  const box = new THREE.Box3(), baseBox = new THREE.Box3(), tableBox = new THREE.Box3();
  const localShoulderBox = new THREE.Box3(
    new THREE.Vector3(-0.85, -0.03, -0.86),
    new THREE.Vector3(2.54, 2.06, 2.35)
  ).expandByScalar(-0.10);
  const pJ3 = new THREE.Vector3(), pJ4 = new THREE.Vector3(), samplePt = new THREE.Vector3();
  const mInvL1 = new THREE.Matrix4(), vIn1 = new THREE.Vector3();
  const targetInverse = new THREE.Matrix4(), meshToTarget = new THREE.Matrix4();
  const baseLocalBox = new THREE.Box3();
  const triangle = new THREE.Triangle(new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3());

  // Test the actual link surface: a long mesh can cross a box while every
  // sampled vertex and the J3–J4 centerline remain outside it.
  const meshIntersectsLocalBox = (mesh, target, targetBox) => {
    const geometry = mesh?.geometry;
    const position = geometry?.attributes.position;
    if (!position) return false;
    if (!geometry.boundingBox) geometry.computeBoundingBox();
    meshToTarget.multiplyMatrices(targetInverse.copy(target.matrixWorld).invert(), mesh.matrixWorld);
    box.copy(geometry.boundingBox).applyMatrix4(meshToTarget);
    if (!box.intersectsBox(targetBox)) return false;
    const indices = geometry.index;
    const count = indices?.count ?? position.count;
    for (let i = 0; i < count; i += 3) {
      triangle.a.fromBufferAttribute(position, indices ? indices.getX(i) : i).applyMatrix4(meshToTarget);
      triangle.b.fromBufferAttribute(position, indices ? indices.getX(i + 1) : i + 1).applyMatrix4(meshToTarget);
      triangle.c.fromBufferAttribute(position, indices ? indices.getX(i + 2) : i + 2).applyMatrix4(meshToTarget);
      if (targetBox.intersectsTriangle(triangle)) return true;
    }
    return false;
  };

  /** 【D】夹爪检测：默认只拦穿透（margin=0），需要提前停时可加大 margin。 */
  const gripperMargin = Number.isFinite(motionOptions.gripperMargin) ? motionOptions.gripperMargin : 0;
  const gripperCheckEnabled = motionOptions.gripperCollision !== false;
  const currentScene = () => resolveGripperScene(api, motionOptions);

  const checkGripper = () => {
    if (!gripperCheckEnabled) return null;
    const { parts, obstacles } = currentScene();
    if (!parts.length) return null;
    return checkGripperCollision(parts, obstacles, {
      margin: gripperMargin,
      maxPairs: motionOptions.maxPairs,
      includeHidden: false,
    });
  };

  /** 记录一次碰撞（供界面与第 5 章统计读取）。 */
  const markCollision = (reason, detail) => {
    state.collision = true;
    state.collisionReason = reason;
    state.collisionDetail = detail ?? null;
    state.collisionCount += 1;
  };

  const isColliding = () => {
    const base = api.robot?.getObjectByName?.('link_base');
    if (!base?.geometry) return false;
    api.robot.updateMatrixWorld(true);

    const floorY = api.floorY ?? (base.getWorldPosition(new THREE.Vector3()).y);
    // 1. 全连杆（link_1 ~ link_5）地面穿模精细化检测：任一几何体触及地面立即安全拦截（预留 0.02 单位安全裕量）
    const allLinks = ['link_1', 'link_2', 'link_3', 'link_4', 'link_5'].map((n) => api.robot.getObjectByName(n)).filter(Boolean);
    for (const link of allLinks) {
      box.setFromObject(link);
      if (!box.isEmpty() && box.min.y < floorY + 0.02) return true;
    }

    const l1 = api.robot.getObjectByName('link_1');
    if (l1) mInvL1.copy(l1.matrixWorld).invert();

    // 2. 前臂与末端执行链（link_4、link_5、tcp）工位、基座及肩部干涉区防护
    baseBox.setFromObject(base).expandByScalar(-0.08);
    const hasTable = api.workstation?.table?.children?.length > 0;
    if (hasTable) tableBox.setFromObject(api.workstation.table).expandByScalar(-0.05);
    const endNodes = ['link_4', 'link_5', 'tcp'].map((n) => api.robot.getObjectByName(n)).filter(Boolean);
    for (const node of endNodes) {
      box.setFromObject(node);
      if (box.isEmpty()) continue;
      if (box.intersectsBox(baseBox)) return true;
      if (hasTable && box.intersectsBox(tableBox)) return true;
      if (l1) {
        node.getWorldPosition(vIn1).applyMatrix4(mInvL1);
        if (localShoulderBox.containsPoint(vIn1)) return true;
      }
    }

    // 3. Check link_3 triangles against the base and shoulder root volumes.
    // Vertex-only sampling and the joint centerline miss face crossings.
    const l3 = api.robot.getObjectByName('link_3');
    if (l3?.geometry) {
      if (!base.geometry.boundingBox) base.geometry.computeBoundingBox();
      baseLocalBox.copy(base.geometry.boundingBox).expandByScalar(-0.02);
      if (meshIntersectsLocalBox(l3, base, baseLocalBox)) return true;
      if (l1 && meshIntersectsLocalBox(l3, l1, localShoulderBox)) return true;
    }

    // 4. Keep the J3–J4 centerline check as a complementary guard:
    const j3 = api.robot.getObjectByName('J3');
    const j4 = api.robot.getObjectByName('J4');
    if (j3 && j4) {
      pJ3.setFromMatrixPosition(j3.matrixWorld);
      pJ4.setFromMatrixPosition(j4.matrixWorld);
      for (let i = 0; i <= 10; i++) {
        samplePt.lerpVectors(pJ3, pJ4, i / 10);
        if (baseBox.distanceToPoint(samplePt) < 0.35) return true;
      }
    }

    // 5. 【D】末端夹爪（掌部 + 左右指 + 已挂载工件）与工位 / 基座 / 未挂载工件的三角面片级检测。
    const gripper = checkGripper();
    if (gripper?.colliding) {
      markCollision(gripper.reason, gripper);
      return true;
    }

    return false;
  };

  return {
    state, stop, allowed, target, busy: () => !!state.jog || state.targets.size > 0 || !!state.segment,
    /** 【D】只读：主动检测一次夹爪碰撞，不移动机械臂（界面与测试用）。 */
    checkGripperCollision: (gripperParts, obstacles, options = {}) => checkGripperCollision(
      gripperParts ?? currentScene().parts,
      obstacles ?? currentScene().obstacles,
      { margin: gripperMargin, ...options },
    ),
    /** 【D】当前夹爪碰撞配置（只读快照）。 */
    getGripperCollisionConfig: () => ({ enabled: gripperCheckEnabled, margin: gripperMargin }),
    moveJ(angles) {
      if (!allowed()) throw new Error('请先解除急停并使能');
      if (!Array.isArray(angles) || angles.length !== api.joints.length || angles.some((v, i) => !Number.isFinite(v) || v < api.joints[i].lower - 1e-9 || v > api.joints[i].upper + 1e-9)) throw new Error('MOVJ 位姿角度无效或超限');
      state.jog = null; state.targets.clear();
      const start = api.joints.map((joint) => joint.angle);
      const distance = Math.max(...angles.map((angle, i) => Math.abs(angle - start[i])));
      state.segment = { start, goal: [...angles], distance, progress: 0 };
    },
    update(delta) {
      if (!allowed()) { if (state.programActive || state.jog || state.targets.size || state.segment) stop(); return; }
      const step = THREE.MathUtils.degToRad(30 * state.speed / 100) * Math.min(Math.max(delta, 0), 0.05);
      if (state.segment) {
        const segment = state.segment;
        const backup = api.joints.map((j) => j.angle);
        segment.progress = segment.distance < 1e-12 ? 1 : Math.min(1, segment.progress + step / (1.5 * segment.distance));
        const t = segment.progress, blend = t * t * (3 - 2 * t);
        api.joints.forEach((joint, i) => apply(joint, segment.start[i] + (segment.goal[i] - segment.start[i]) * blend));
        if (isColliding()) {
          api.joints.forEach((joint, i) => apply(joint, backup[i]));
          stop(); state.collision = true;
        } else if (t === 1) state.segment = null;
      } else if (state.jog) {
        const joint = api.joints.find((j) => j.name === state.jog.name);
        const prev = joint.angle;
        apply(joint, joint.angle + state.jog.direction * step);
        if (isColliding()) { apply(joint, prev); state.jog = null; state.collision = true; }
      } else for (const [name, goal] of state.targets) {
        const joint = api.joints.find((j) => j.name === name), difference = goal - joint.angle;
        const prev = joint.angle;
        apply(joint, joint.angle + Math.sign(difference) * Math.min(Math.abs(difference), step));
        if (isColliding()) { apply(joint, prev); state.targets.delete(name); state.collision = true; }
        else if (Math.abs(goal - joint.angle) < 1e-9) state.targets.delete(name);
      }
      api.robot.updateMatrixWorld(true); urdfModel.root.updateMatrixWorld(true);
    },
  };
}
