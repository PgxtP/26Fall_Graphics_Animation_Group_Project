import * as THREE from 'three';

/**
 * 【5 号位 · 组员 D】WorkspaceCloud —— 蒙特卡洛采样与可达工作空间点云分析
 *
 * 对外接口（与仓库骨架一致，方法名与返回结构未改动）：
 *   WorkspaceCloud.generatePointCloud(fkPositionFn, limits, sampleCount)  → THREE.Points
 *   WorkspaceCloud.analyzeWorkspaceGeometry(pointCloudObject)             → { reachRadiusMax, reachRadiusMin, zHeightRange, evaluationText }
 *   WorkspaceCloud.detectCavities(pointCloudObject, options)              → 额外新增：空腔率与空洞清单
 *
 * 集成方式（组长在 main.js / RobotModel 里只需要两行）：
 *   1) const cloud = WorkspaceCloud.generatePointCloud(
 *        (angles) => Kinematics.forwardKinematics(angles),   // 1 号位的纯数学 FK，返回 Vector3
 *        jointLimits,                                        // [{min,max}] 弧度
 *        20000);
 *      scene.add(cloud);
 *   2) const report = WorkspaceCloud.analyzeWorkspaceGeometry(cloud);   // 直接拿去写报告
 *
 * 设计约束：
 *   · 关节数不写死：按 limits.length 决定采几维（5 关节、6 关节都能跑）；
 *   · 单位不换算：传入什么单位就统计什么单位，evaluationText 里的单位由 unitLabel 控制（默认 m）；
 *   · 除 three 外零依赖，不引用任何其他组员的文件；
 *   · 采样可复现：第 4 个可选参数 { seed } 固定后，每次生成的点云完全一致（写报告时可复现）。
 */

const DEG2RAD = Math.PI / 180;

/** 可复现随机数（同一 seed 得到同一点云） */
function mulberry32(seed) {
  let a = seed >>> 0;
  return function random() {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * 统一限位格式：支持 { min, max }（弧度，仓库约定）与 [min, max]（弧度；
 * 若数值明显超出 ±2π 则按角度处理，方便直接喂 5 关节那套 -120~120 的限位）。
 */
function normalizeLimit(entry) {
  if (Array.isArray(entry)) {
    const lo = Number(entry[0]);
    const hi = Number(entry[1]);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) throw new Error('关节限位必须是有限数');
    const looksLikeDegrees = Math.max(Math.abs(lo), Math.abs(hi)) > 2 * Math.PI + 1e-6;
    const factor = looksLikeDegrees ? DEG2RAD : 1;
    return [Math.min(lo, hi) * factor, Math.max(lo, hi) * factor];
  }
  if (entry && Number.isFinite(Number(entry.min)) && Number.isFinite(Number(entry.max))) {
    return [Math.min(Number(entry.min), Number(entry.max)), Math.max(Number(entry.min), Number(entry.max))];
  }
  throw new Error('关节限位格式无效：应为 {min,max} 或 [min,max]');
}

/** 兼容三种 FK 返回：THREE.Vector3 / {x,y,z} / [x,y,z] */
function readPoint(result) {
  if (!result) return null;
  if (typeof result.x === 'number' && typeof result.y === 'number' && typeof result.z === 'number') {
    return [result.x, result.y, result.z];
  }
  if (Array.isArray(result) || ArrayBuffer.isView(result)) {
    if (!Number.isFinite(result[0])) return null;
    return [result[0], result[1], result[2]];
  }
  if (result.position && typeof result.position.x === 'number') {
    return [result.position.x, result.position.y, result.position.z];
  }
  return null;
}

function countToGradient(value, stops) {
  const t = Math.min(1, Math.max(0, value));
  const segments = stops.length - 1;
  const position = t * segments;
  const index = Math.min(segments - 1, Math.floor(position));
  const k = position - index;
  const [r1, g1, b1] = stops[index];
  const [r2, g2, b2] = stops[index + 1];
  return [r1 + (r2 - r1) * k, g1 + (g2 - g1) * k, b1 + (b2 - b1) * k];
}

export class WorkspaceCloud {
  /**
   * 蒙特卡洛随机采样生成正向运动学 TCP 点云
   * @param {(angles: number[]) => THREE.Vector3} fkPositionFn 1 号位提供的纯数学 FK 位置函数
   * @param {Array<{ min: number, max: number }>} limits 关节弧度限位
   * @param {number} [sampleCount=20000] 采样点数
   * @param {{seed?:number, size?:number, colorMode?:'height'|'radius'|'flat'}} [options]
   * @returns {THREE.Points} 包含数万个顶点的 Three.js 点云对象
   */
  static generatePointCloud(fkPositionFn, limits, sampleCount = 20000, options = {}) {
    if (typeof fkPositionFn !== 'function') throw new Error('generatePointCloud 需要传入 FK 位置函数');
    if (!Array.isArray(limits) || !limits.length) throw new Error('generatePointCloud 需要关节限位数组');

    const count = Math.max(1, Math.floor(Number(sampleCount) || 0));
    const ranges = limits.map(normalizeLimit);
    const jointCount = ranges.length;
    const random = mulberry32(Number.isFinite(options.seed) ? options.seed : (Date.now() & 0xffffffff));
    const angles = new Array(jointCount).fill(0);

    const positions = new Float32Array(count * 3);
    let written = 0;
    let attempts = 0;
    const maxAttempts = count * 4 + 1000;

    while (written < count && attempts < maxAttempts) {
      attempts += 1;
      for (let index = 0; index < jointCount; index += 1) {
        const [lo, hi] = ranges[index];
        angles[index] = lo + random() * (hi - lo);
      }
      const point = readPoint(fkPositionFn(angles));
      if (!point) continue;
      if (!Number.isFinite(point[0]) || !Number.isFinite(point[1]) || !Number.isFinite(point[2])) continue;
      positions[written * 3] = point[0];
      positions[written * 3 + 1] = point[1];
      positions[written * 3 + 2] = point[2];
      written += 1;
    }

    const positionArray = written === count ? positions : positions.slice(0, written * 3);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positionArray, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(WorkspaceCloud.computeColors(positionArray, options.colorMode ?? 'height'), 3));
    geometry.computeBoundingSphere();

    const material = new THREE.PointsMaterial({
      size: Number.isFinite(options.size) ? options.size : 0.015,
      color: 0x4f86f7,
      vertexColors: (options.colorMode ?? 'height') !== 'flat',
      transparent: true,
      opacity: 0.85,
      depthWrite: false,
      sizeAttenuation: true,
    });

    const points = new THREE.Points(geometry, material);
    points.name = 'workspace-cloud';
    points.userData = { sampleCount: written, requested: count, seed: options.seed ?? null };
    return points;
  }

  /**
   * 分析工作空间可达边界与不可达空洞
   * @param {THREE.Points} pointCloudObject 采样点云对象
   * @param {{unitLabel?:string, voxel?:number}} [options]
   * @returns {{reachRadiusMax:number, reachRadiusMin:number, zHeightRange:[number,number], evaluationText:string}}
   */
  static analyzeWorkspaceGeometry(pointCloudObject, options = {}) {
    const positions = WorkspaceCloud.readPositions(pointCloudObject);
    const unit = options.unitLabel ?? 'm';

    if (!positions.length) {
      return {
        reachRadiusMax: 0,
        reachRadiusMin: 0,
        zHeightRange: [0, 0],
        evaluationText: '点云为空：请先调用 generatePointCloud() 生成采样点。',
      };
    }

    const count = positions.length / 3;
    let radiusMin = Infinity;
    let radiusMax = 0;
    let heightMin = Infinity;
    let heightMax = -Infinity;

    for (let index = 0; index < count; index += 1) {
      const x = positions[index * 3];
      const y = positions[index * 3 + 1];
      const z = positions[index * 3 + 2];
      const radius = Math.hypot(x, z); // three.js 约定：Y 轴向上，水平面是 XZ
      radiusMin = Math.min(radiusMin, radius);
      radiusMax = Math.max(radiusMax, radius);
      heightMin = Math.min(heightMin, y);
      heightMax = Math.max(heightMax, y);
    }

    const cavities = WorkspaceCloud.detectCavities(pointCloudObject, options);
    const fix = (value) => Number(value).toFixed(3);
    const lines = [
      `采样点数：${count}`,
      `可达半径（水平）：${fix(radiusMin)} ~ ${fix(radiusMax)} ${unit}`,
      `高度范围：${fix(heightMin)} ~ ${fix(heightMax)} ${unit}`,
      `空腔占比：${(cavities.voidRatio * 100).toFixed(1)}%（${cavities.cols}×${cavities.rows} 栅格）`,
    ];

    if (cavities.largest) {
      lines.push(
        `最大空洞：${cavities.largest.cells} 格，半径 ${fix(cavities.largest.radiusRange[0])} ~ ${fix(cavities.largest.radiusRange[1])} ${unit}，`
        + `高度 ${fix(cavities.largest.heightRange[0])} ~ ${fix(cavities.largest.heightRange[1])} ${unit}`,
      );
    } else {
      lines.push('最大空洞：未检出（说明该工作空间在水平截面上连续）');
    }

    lines.push(
      `工位布置建议：常用作业点应落在半径 ${fix(radiusMin * 1.05)} ~ ${fix(radiusMax * 0.95)} ${unit}、`
      + `高度 ${fix(heightMin + (heightMax - heightMin) * 0.2)} ~ ${fix(heightMax - (heightMax - heightMin) * 0.1)} ${unit} 区间内；`
      + '若落点在空洞范围内，需调整工件位置或改变作业姿态。',
    );

    return {
      reachRadiusMax: radiusMax,
      reachRadiusMin: radiusMin,
      zHeightRange: [heightMin, heightMax],
      evaluationText: lines.join('\n'),
    };
  }

  /**
   * 空洞识别：把点云投影到 (水平半径 r, 高度 y) 平面并栅格化，
   * 同一高度行内被两侧点夹住的空格记为空洞，再做 4 邻域连通域聚类。
   *
   * @param {THREE.Points} pointCloudObject
   * @param {{voxel?:number, minCells?:number, unitLabel?:string}} [options]
   * @returns {{voxel:number, cols:number, rows:number, occupiedCells:number, voidCells:number, voidRatio:number, holes:Array, largest:object|null}}
   */
  static detectCavities(pointCloudObject, options = {}) {
    const positions = WorkspaceCloud.readPositions(pointCloudObject);
    if (!positions.length) {
      return { voxel: 0, cols: 0, rows: 0, occupiedCells: 0, voidCells: 0, voidRatio: 0, holes: [], largest: null };
    }

    const count = positions.length / 3;
    let radiusMin = Infinity;
    let radiusMax = 0;
    let heightMin = Infinity;
    let heightMax = -Infinity;

    for (let index = 0; index < count; index += 1) {
      const radius = Math.hypot(positions[index * 3], positions[index * 3 + 2]);
      const y = positions[index * 3 + 1];
      radiusMin = Math.min(radiusMin, radius);
      radiusMax = Math.max(radiusMax, radius);
      heightMin = Math.min(heightMin, y);
      heightMax = Math.max(heightMax, y);
    }

    // 格子边长默认取工作空间最大跨度的 1/80，避免单位不同时格子过大或过小
    const extent = Math.max(radiusMax - radiusMin, heightMax - heightMin, 1e-6);
    const voxel = Number.isFinite(options.voxel) && options.voxel > 0 ? options.voxel : extent / 80;
    const minCells = Number.isFinite(options.minCells) ? options.minCells : 4;

    const cols = Math.max(1, Math.ceil((radiusMax - radiusMin) / voxel) + 1);
    const rows = Math.max(1, Math.ceil((heightMax - heightMin) / voxel) + 1);
    const occupied = new Uint8Array(cols * rows);
    const voidMask = new Uint8Array(cols * rows);

    for (let index = 0; index < count; index += 1) {
      const radius = Math.hypot(positions[index * 3], positions[index * 3 + 2]);
      const y = positions[index * 3 + 1];
      const col = Math.min(cols - 1, Math.max(0, Math.floor((radius - radiusMin) / voxel)));
      const row = Math.min(rows - 1, Math.max(0, Math.floor((y - heightMin) / voxel)));
      occupied[row * cols + col] = 1;
    }

    let occupiedCells = 0;
    for (let row = 0; row < rows; row += 1) {
      const base = row * cols;
      let first = -1;
      let last = -1;
      for (let col = 0; col < cols; col += 1) {
        if (!occupied[base + col]) continue;
        if (first < 0) first = col;
        last = col;
        occupiedCells += 1;
      }
      if (first < 0) continue;
      for (let col = first; col <= last; col += 1) {
        if (!occupied[base + col]) voidMask[base + col] = 1;
      }
    }

    const visited = new Uint8Array(cols * rows);
    const holes = [];
    const stack = [];
    let voidCells = 0;

    for (let index = 0; index < voidMask.length; index += 1) {
      if (!voidMask[index]) continue;
      voidCells += 1;
      if (visited[index]) continue;

      stack.length = 0;
      stack.push(index);
      visited[index] = 1;

      let cells = 0;
      let colMin = Infinity;
      let colMax = -Infinity;
      let rowMin = Infinity;
      let rowMax = -Infinity;

      while (stack.length) {
        const current = stack.pop();
        const row = Math.floor(current / cols);
        const col = current % cols;
        cells += 1;
        colMin = Math.min(colMin, col);
        colMax = Math.max(colMax, col);
        rowMin = Math.min(rowMin, row);
        rowMax = Math.max(rowMax, row);

        if (col > 0) {
          const next = current - 1;
          if (voidMask[next] && !visited[next]) { visited[next] = 1; stack.push(next); }
        }
        if (col < cols - 1) {
          const next = current + 1;
          if (voidMask[next] && !visited[next]) { visited[next] = 1; stack.push(next); }
        }
        if (row > 0) {
          const next = current - cols;
          if (voidMask[next] && !visited[next]) { visited[next] = 1; stack.push(next); }
        }
        if (row < rows - 1) {
          const next = current + cols;
          if (voidMask[next] && !visited[next]) { visited[next] = 1; stack.push(next); }
        }
      }

      if (cells >= minCells) {
        holes.push({
          cells,
          radiusRange: [radiusMin + colMin * voxel, radiusMin + colMax * voxel],
          heightRange: [heightMin + rowMin * voxel, heightMin + rowMax * voxel],
        });
      }
    }

    holes.sort((a, b) => b.cells - a.cells);
    const total = occupiedCells + voidCells;

    return {
      voxel,
      cols,
      rows,
      occupiedCells,
      voidCells,
      voidRatio: total ? voidCells / total : 0,
      holes,
      largest: holes[0] ?? null,
    };
  }

  /** 从 THREE.Points 里取出 position 数组（兼容不同构造方式） */
  static readPositions(pointCloudObject) {
    const attribute = pointCloudObject?.geometry?.attributes?.position;
    if (!attribute) return new Float32Array(0);
    return attribute.array;
  }

  /** 顶点颜色：按高度（Y）或水平半径着色 */
  static computeColors(positionArray, mode = 'height') {
    const count = positionArray.length / 3;
    const colors = new Float32Array(count * 3);
    if (!count) return colors;
    if (mode === 'flat') {
      for (let index = 0; index < count; index += 1) {
        colors[index * 3] = 0.31;
        colors[index * 3 + 1] = 0.53;
        colors[index * 3 + 2] = 0.97;
      }
      return colors;
    }

    const valueAt = (index) => (mode === 'radius'
      ? Math.hypot(positionArray[index * 3], positionArray[index * 3 + 2])
      : positionArray[index * 3 + 1]);

    let min = Infinity;
    let max = -Infinity;
    for (let index = 0; index < count; index += 1) {
      const value = valueAt(index);
      min = Math.min(min, value);
      max = Math.max(max, value);
    }
    const span = max - min || 1;
    const stops = mode === 'radius'
      ? [[0.09, 0.26, 0.43], [0.30, 0.71, 0.89], [0.78, 0.94, 1.0]]
      : [[0.14, 0.24, 0.47], [0.22, 0.65, 0.86], [0.48, 0.89, 0.75], [1.0, 0.77, 0.42]];

    for (let index = 0; index < count; index += 1) {
      const [r, g, b] = countToGradient((valueAt(index) - min) / span, stops);
      colors[index * 3] = r;
      colors[index * 3 + 1] = g;
      colors[index * 3 + 2] = b;
    }
    return colors;
  }
}
