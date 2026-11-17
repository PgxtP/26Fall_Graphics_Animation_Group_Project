import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';

export function createMotion(api, urdfModel) {
  const state = { enabled: false, estop: false, speed: 25, jog: null, targets: new Map(), segment: null, programActive: false, stopVersion: 0, collision: false };
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

    return false;
  };

  return { state, stop, allowed, target, busy: () => !!state.jog || state.targets.size > 0 || !!state.segment,
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
