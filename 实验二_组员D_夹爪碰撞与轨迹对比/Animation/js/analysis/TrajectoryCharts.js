/**
 * 【王静】TrajectoryCharts —— 末端轨迹记录、图表绘制与 CSV 导出
 *
 * 对外接口（与仓库骨架一致，未改动任何签名）：
 *   const charts = new TrajectoryCharts();
 *   charts.recordSample(timestamp, tcpPosition, linearVelocity);  // 动画循环里高频调用
 *   charts.renderCharts(chartContainer);                          // 传入一个空的 DOM 容器
 *   charts.exportToCSV();                                         // 返回 CSV 字符串
 *   charts.getStats();                                            // 额外提供：最大/平均速度、轨迹长度
 *
 * 集成方式（谷昔桐在 main.js 里只需要三行）：
 *   1) import { TrajectoryCharts } from './analysis/TrajectoryCharts.js';
 *   2) 在动画循环里： charts.recordSample(elapsed, tcp.getWorldPosition(v), speed);
 *   3) 初始化一次：  charts.renderCharts(document.getElementById('trajectory-chart'));
 *   —— 容器为空即可，canvas 由本模块自己创建，页面结构不用改。
 *
 * 设计约束：
 *   · 不写死关节数（5 或 6 都行），本文件不关心机器人结构；
 *   · 不做单位换算：传进来是什么单位就画什么，标签默认 'm'，可用
 *     new TrajectoryCharts({ unitLabel: '模型单位' }) 改；
 *   · 零依赖：不 import three，不依赖其他组员的任何文件。
 */

const DARK = {
  background: '#0f1c26',
  panel: 'rgba(18, 32, 43, 0.9)',
  border: '#2b3f50',
  grid: 'rgba(140, 170, 195, 0.14)',
  text: '#a9bccd',
  title: '#dce8f2',
  x: '#6fb3ff',
  y: '#65d9c5',
  z: '#ffc56a',
  speed: '#ff8fb1',
  path: '#7fd3ff',
  start: '#65d9c5',
  end: '#ff8a8a',
};

const LIGHT = {
  background: '#ffffff',
  panel: 'rgba(246, 249, 252, 0.95)',
  border: '#d3dde7',
  grid: 'rgba(90, 120, 150, 0.16)',
  text: '#4a6076',
  title: '#1f3b57',
  x: '#2f7fd8',
  y: '#0f9c85',
  z: '#c8811a',
  speed: '#d24f7f',
  path: '#2f8fd8',
  start: '#0f9c85',
  end: '#d95050',
};

const FONT = '"Microsoft YaHei", "PingFang SC", system-ui, sans-serif';
const DEFAULT_HEIGHT = 320;
const COMPARE_HEIGHT = 560; // 对比视图三张图的默认高度
const JOINT_COLORS = ['#6fb3ff', '#65d9c5', '#ffc56a', '#ff8fb1', '#b48bff', '#8fd06f'];

function parseCssColor(value) {
  if (!value) return null;
  const match = value.match(/rgba?\(\s*([\d.]+)[,\s]+([\d.]+)[,\s]+([\d.]+)(?:[,\s/]+([\d.]+))?/i);
  if (!match) return null;
  return [Number(match[1]), Number(match[2]), Number(match[3]), match[4] === undefined ? 1 : Number(match[4])];
}

function pickTheme(container) {
  try {
    const body = getComputedStyle(document.body);
    const sample = parseCssColor(getComputedStyle(container).backgroundColor) ?? parseCssColor(body.backgroundColor);
    if (!sample || sample[3] === 0) return DARK;
    const luminance = 0.299 * sample[0] + 0.587 * sample[1] + 0.114 * sample[2];
    return luminance > 150 ? LIGHT : DARK;
  } catch {
    return DARK;
  }
}

function finiteOr(value, fallback) {
  return typeof value === 'number' && Number.isFinite(value) ? value : fallback;
}

function readComponent(source, key, index) {
  if (source == null) return 0;
  if (typeof source === 'number') return source;
  const byKey = source[key];
  if (typeof byKey === 'number') return byKey;
  const byIndex = source[index];
  return typeof byIndex === 'number' ? byIndex : 0;
}

function niceTicks(min, max, count) {
  if (!Number.isFinite(min) || !Number.isFinite(max) || !(max > min)) {
    return { min: 0, max: 1, ticks: [0, 1] };
  }
  const raw = (max - min) / Math.max(2, count || 5);
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / magnitude;
  const step = (norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 2.5 ? 2.5 : norm <= 5 ? 5 : 10) * magnitude;
  const lo = Math.floor(min / step) * step;
  const hi = Math.ceil(max / step) * step;
  const ticks = [];
  for (let value = lo; value <= hi + step * 1e-6; value += step) {
    ticks.push(Math.abs(value) < step * 1e-9 ? 0 : Number(value.toFixed(10)));
  }
  return { min: lo, max: hi, ticks };
}

function roundRect(ctx, x, y, width, height, radius) {
  if (typeof ctx.roundRect === 'function') {
    ctx.beginPath();
    ctx.roundRect(x, y, width, height, radius);
    return;
  }
  const r = Math.min(radius, width / 2, height / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + width, y, x + width, y + height, r);
  ctx.arcTo(x + width, y + height, x, y + height, r);
  ctx.arcTo(x, y + height, x, y, r);
  ctx.arcTo(x, y, x + width, y, r);
  ctx.closePath();
}

export class TrajectoryCharts {
  /**
   * @param {{maxSamples?:number, windowSeconds?:number, unitLabel?:string, follow?:boolean}} [options]
   */
  constructor(options = {}) {
    /** @type {Array<{t:number,x:number,y:number,z:number,v:number}>} */
    this.samples = [];
    this.maxSamples = options.maxSamples ?? 20000;
    this.windowSeconds = options.windowSeconds ?? 10;
    this.unitLabel = options.unitLabel ?? 'm';
    this.follow = options.follow !== false;

    this.container = null;
    this.canvas = null;
    this.ctx = null;
    this.theme = DARK;
    this.resizeObserver = null;
    this._raf = 0;

    // 【王静 · 组员 D】实验二增量：关节角采样、MOVL 规划结果、分段标签与对比视图。
    /** @type {Array<{t:number,joints:number[],tag:string}>} 逐帧关节角（度） */
    this.jointSamples = [];
    /** 最近一次 MOVL 规划结果（含 dampingUsed / manipulability / singular） */
    this.plan = null;
    /** 当前采样标签：'MOVJ' 或 'MOVL'，由 analysis-panel 按执行器状态切换 */
    this.activeTag = 'MOVJ';
    /** buildComparison() 的结果 */
    this.comparison = null;
    /** 对比视图的 canvas */
    this.compareContainer = null;
    this.compareCanvas = null;
    this.compareCtx = null;
  }

  get count() {
    return this.samples.length;
  }

  /**
   * 记录一个采样点（由动画循环定期调用）
   * @param {number} timestamp 当前仿真运行时间（秒）
   * @param {{x:number,y:number,z:number}|number[]} tcpPosition TCP 世界坐标
   * @param {number|{x:number,y:number,z:number}|number[]} linearVelocity 末端线速度大小
   */
  recordSample(timestamp, tcpPosition, linearVelocity) {
    const time = finiteOr(timestamp, this.samples.length ? this.samples.at(-1).t : 0);
    const x = finiteOr(readComponent(tcpPosition, 'x', 0), 0);
    const y = finiteOr(readComponent(tcpPosition, 'y', 1), 0);
    const z = finiteOr(readComponent(tcpPosition, 'z', 2), 0);

    let speed = 0;
    if (typeof linearVelocity === 'number') {
      speed = finiteOr(linearVelocity, 0);
    } else if (linearVelocity) {
      speed = Math.hypot(
        readComponent(linearVelocity, 'x', 0),
        readComponent(linearVelocity, 'y', 1),
        readComponent(linearVelocity, 'z', 2),
      );
    } else {
      const previous = this.samples.at(-1);
      if (previous) {
        const dt = time - previous.t;
        speed = dt > 1e-9 ? Math.hypot(x - previous.x, y - previous.y, z - previous.z) / dt : 0;
      }
    }

    this.samples.push({ t: time, x, y, z, v: Math.abs(speed), tag: this.activeTag });

    const overflow = this.samples.length - this.maxSamples;
    if (overflow > 200) this.samples.splice(0, overflow);
  }

  /** 统计指标：最大/平均速度、轨迹长度、时长（报告用） */
  getStats() {
    const list = this.samples;
    if (!list.length) {
      return { count: 0, duration: 0, maxSpeed: 0, avgSpeed: 0, pathLength: 0 };
    }
    let maxSpeed = 0;
    let sumSpeed = 0;
    let pathLength = 0;
    for (let index = 0; index < list.length; index += 1) {
      const point = list[index];
      maxSpeed = Math.max(maxSpeed, point.v);
      sumSpeed += point.v;
      if (index > 0) {
        const previous = list[index - 1];
        pathLength += Math.hypot(point.x - previous.x, point.y - previous.y, point.z - previous.z);
      }
    }
    return {
      count: list.length,
      duration: list.at(-1).t - list[0].t,
      maxSpeed,
      avgSpeed: sumSpeed / list.length,
      pathLength,
    };
  }

  clear() {
    this.samples.length = 0;
    this._scheduleDraw();
  }

  /**
   * 在容器内创建 canvas 并绘制图表；容器为空即可，页面结构不用改。
   * @param {HTMLElement} chartContainer
   */
  renderCharts(chartContainer) {
    if (!chartContainer) return;
    this.container = chartContainer;
    this.theme = pickTheme(chartContainer);

    let canvas = chartContainer.querySelector('canvas.trajectory-chart-canvas');
    if (!canvas) {
      canvas = document.createElement('canvas');
      canvas.className = 'trajectory-chart-canvas';
      canvas.setAttribute('role', 'img');
      canvas.setAttribute('aria-label', '末端轨迹、位置与速度曲线');
      canvas.style.display = 'block';
      canvas.style.width = '100%';
      chartContainer.appendChild(canvas);
    }
    if (!chartContainer.style.height && !canvas.style.height) {
      canvas.style.height = `${DEFAULT_HEIGHT}px`;
    }
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');

    if (this.resizeObserver) this.resizeObserver.disconnect();
    if (typeof ResizeObserver === 'function') {
      this.resizeObserver = new ResizeObserver(() => this._scheduleDraw());
      this.resizeObserver.observe(chartContainer);
    }
    this._scheduleDraw();
  }

  /** 动画循环里可以每帧调用，内部已做节流 */
  update() {
    this._scheduleDraw();
  }

  _scheduleDraw() {
    if (this._raf) return;
    const raf = typeof requestAnimationFrame === 'function' ? requestAnimationFrame : (fn) => setTimeout(fn, 16);
    this._raf = raf(() => {
      this._raf = 0;
      this.draw();
    });
  }

  /** 直接绘制（renderCharts 之后由内部调用，也可以手动调） */
  draw() {
    const canvas = this.canvas;
    const ctx = this.ctx;
    if (!canvas || !ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth || 860;
    const height = canvas.clientHeight || DEFAULT_HEIGHT;
    const pixelWidth = Math.max(1, Math.round(width * dpr));
    const pixelHeight = Math.max(1, Math.round(height * dpr));
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = this.theme.background;
    ctx.fillRect(0, 0, width, height);

    const pad = 10;
    const gap = 10;
    const leftWidth = Math.max(140, Math.round((width - pad * 2 - gap) * 0.34));
    const rightX = pad + leftWidth + gap;
    const rightWidth = width - rightX - pad;
    const halfHeight = Math.floor((height - pad * 2 - gap) / 2);

    const rectPath = { x: pad, y: pad, w: leftWidth, h: height - pad * 2 };
    const rectPos = { x: rightX, y: pad, w: rightWidth, h: halfHeight };
    const rectSpeed = { x: rightX, y: pad + halfHeight + gap, w: rightWidth, h: halfHeight };

    const unit = this.unitLabel;
    if (!this.samples.length) {
      this._panel(ctx, rectPath, { title: '末端轨迹 · XZ 俯视', empty: '暂无数据' });
      this._panel(ctx, rectPos, { title: `末端位置 p(t) · ${unit}`, empty: '暂无数据' });
      this._panel(ctx, rectSpeed, { title: `末端线速度 v(t) · ${unit}/s`, empty: '暂无数据' });
      return;
    }

    const last = this.samples.at(-1);
    const tEnd = last.t;
    const tStart = this.follow ? Math.max(this.samples[0].t, tEnd - this.windowSeconds) : this.samples[0].t;
    const visible = this.samples.slice(this._startIndex(tStart));

    let minPosition = Infinity;
    let maxPosition = -Infinity;
    let maxSpeed = 0;
    let minX = Infinity;
    let maxX = -Infinity;
    let minZ = Infinity;
    let maxZ = -Infinity;

    for (const point of visible) {
      minPosition = Math.min(minPosition, point.x, point.y, point.z);
      maxPosition = Math.max(maxPosition, point.x, point.y, point.z);
      maxSpeed = Math.max(maxSpeed, point.v);
      minX = Math.min(minX, point.x);
      maxX = Math.max(maxX, point.x);
      minZ = Math.min(minZ, point.z);
      maxZ = Math.max(maxZ, point.z);
    }

    const padPosition = Math.max(0.002, (maxPosition - minPosition) * 0.08);
    const positionTicks = niceTicks(minPosition - padPosition, maxPosition + padPosition, 5);
    const speedTicks = niceTicks(0, Math.max(maxSpeed * 1.12, 0.01), 4);
    const timeTicks = niceTicks(tStart, Math.max(tEnd, tStart + 0.5), 5);

    const spanX = Math.max(maxX - minX, 0.02);
    const spanZ = Math.max(maxZ - minZ, 0.02);
    const span = Math.max(spanX, spanZ) * 1.2;
    const centerX = (minX + maxX) / 2;
    const centerZ = (minZ + maxZ) / 2;

    this._panel(ctx, rectPath, {
      title: '末端轨迹 · XZ 俯视',
      xTicks: niceTicks(centerX - span / 2, centerX + span / 2, 4),
      yTicks: niceTicks(centerZ - span / 2, centerZ + span / 2, 4),
      xFormat: (value) => value.toFixed(2),
      yFormat: (value) => value.toFixed(2),
      series: [{ name: '轨迹', color: this.theme.path, points: visible.map((point) => [point.x, point.z]) }],
      markers: [
        { color: this.theme.start, point: [visible[0].x, visible[0].z] },
        { color: this.theme.end, point: [last.x, last.z] },
      ],
    });

    this._panel(ctx, rectPos, {
      title: `末端位置 p(t) · ${unit}`,
      xTicks: timeTicks,
      yTicks: positionTicks,
      xFormat: (value) => value.toFixed(1),
      yFormat: (value) => value.toFixed(2),
      series: [
        { name: 'X', color: this.theme.x, points: visible.map((point) => [point.t, point.x]) },
        { name: 'Y', color: this.theme.y, points: visible.map((point) => [point.t, point.y]) },
        { name: 'Z', color: this.theme.z, points: visible.map((point) => [point.t, point.z]) },
      ],
    });

    this._panel(ctx, rectSpeed, {
      title: `末端线速度 v(t) · ${unit}/s`,
      xTicks: timeTicks,
      yTicks: speedTicks,
      xFormat: (value) => value.toFixed(1),
      yFormat: (value) => value.toFixed(3),
      series: [{ name: 'v', color: this.theme.speed, fill: true, points: visible.map((point) => [point.t, point.v]) }],
    });
  }

  _startIndex(t) {
    const list = this.samples;
    let lo = 0;
    let hi = list.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (list[mid].t < t) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  _panel(ctx, rect, config) {
    const { x, y, w, h } = rect;
    if (w < 60 || h < 50) return;
    const theme = this.theme;

    roundRect(ctx, x + 0.5, y + 0.5, w - 1, h - 1, 6);
    ctx.fillStyle = theme.panel;
    ctx.fill();
    ctx.strokeStyle = theme.border;
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = theme.title;
    ctx.font = `600 11px ${FONT}`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(config.title, x + 10, y + 16);

    if (config.empty) {
      ctx.fillStyle = theme.text;
      ctx.font = `12px ${FONT}`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(config.empty, x + w / 2, y + h / 2);
      return;
    }

    const plot = { x: x + 46, y: y + 26, w: w - 58, h: h - 26 - 22 };
    if (plot.w < 18 || plot.h < 18) return;

    const xt = config.xTicks;
    const yt = config.yTicks;
    const mapX = (value) => plot.x + ((value - xt.min) / (xt.max - xt.min || 1)) * plot.w;
    const mapY = (value) => plot.y + plot.h - ((value - yt.min) / (yt.max - yt.min || 1)) * plot.h;

    // 奇异区等区间着色带（画在网格之下）
    if (config.bands?.length) {
      for (const band of config.bands) {
        const from = mapX(Math.max(xt.min, band.from));
        const to = mapX(Math.min(xt.max, band.to));
        if (!(to > from)) continue;
        ctx.fillStyle = band.color ?? 'rgba(255, 138, 138, 0.18)';
        ctx.fillRect(from, plot.y, to - from, plot.h);
      }
    }

    ctx.save();
    ctx.strokeStyle = theme.grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (const tick of yt.ticks) {
      const py = Math.round(mapY(tick)) + 0.5;
      ctx.moveTo(plot.x, py);
      ctx.lineTo(plot.x + plot.w, py);
    }
    for (const tick of xt.ticks) {
      const px = Math.round(mapX(tick)) + 0.5;
      ctx.moveTo(px, plot.y);
      ctx.lineTo(px, plot.y + plot.h);
    }
    ctx.stroke();
    ctx.restore();

    ctx.fillStyle = theme.text;
    ctx.font = `10px ${FONT}`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const tick of yt.ticks) {
      const py = mapY(tick);
      if (py < plot.y - 1 || py > plot.y + plot.h + 1) continue;
      ctx.fillText(config.yFormat(tick), plot.x - 6, py);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (const tick of xt.ticks) {
      const px = mapX(tick);
      if (px < plot.x - 1 || px > plot.x + plot.w + 1) continue;
      ctx.fillText(config.xFormat(tick), px, plot.y + plot.h + 6);
    }

    if (config.axisHint) {
      ctx.fillStyle = theme.text;
      ctx.font = `10px ${FONT}`;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'bottom';
      ctx.fillText(config.axisHint, plot.x + plot.w, y + h - 4);
    }

    ctx.save();
    ctx.beginPath();
    ctx.rect(plot.x, plot.y - 2, plot.w, plot.h + 4);
    ctx.clip();
    for (const item of config.series ?? []) {
      const points = item.points;
      if (!points || points.length < 2) continue;
      ctx.beginPath();
      points.forEach(([px, py], index) => {
        if (index === 0) ctx.moveTo(mapX(px), mapY(py));
        else ctx.lineTo(mapX(px), mapY(py));
      });
      ctx.strokeStyle = item.color;
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();
      if (item.fill) {
        ctx.lineTo(mapX(points.at(-1)[0]), mapY(yt.min));
        ctx.lineTo(mapX(points[0][0]), mapY(yt.min));
        ctx.closePath();
        const gradient = ctx.createLinearGradient(0, plot.y, 0, plot.y + plot.h);
        gradient.addColorStop(0, 'rgba(255, 143, 177, 0.28)');
        gradient.addColorStop(1, 'rgba(255, 143, 177, 0.02)');
        ctx.fillStyle = gradient;
        ctx.fill();
      }
    }
    ctx.restore();

    for (const marker of config.markers ?? []) {
      const mx = mapX(marker.point[0]);
      const my = mapY(marker.point[1]);
      if (mx < plot.x || mx > plot.x + plot.w || my < plot.y || my > plot.y + plot.h) continue;
      ctx.beginPath();
      ctx.arc(mx, my, 3, 0, Math.PI * 2);
      ctx.fillStyle = marker.color;
      ctx.fill();
      ctx.strokeStyle = theme.background;
      ctx.lineWidth = 1;
      ctx.stroke();
    }

    const series = config.series ?? [];
    if (series.length) {
      ctx.font = `10px ${FONT}`;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      let cursor = x + w - 10;
      for (let index = series.length - 1; index >= 0; index -= 1) {
        const item = series[index];
        cursor -= ctx.measureText(item.name).width;
        ctx.fillStyle = theme.text;
        ctx.fillText(item.name, cursor, y + 15);
        cursor -= 13;
        ctx.strokeStyle = item.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cursor, y + 15);
        ctx.lineTo(cursor + 9, y + 15);
        ctx.stroke();
        cursor -= 12;
      }
    }
  }

  /** 切换采样标签（'MOVJ' / 'MOVL'），供 MOVJ 与 MOVL 同场对比。 */
  setTag(tag) {
    this.activeTag = tag === 'MOVL' ? 'MOVL' : 'MOVJ';
    return this.activeTag;
  }

  /**
   * 记录一帧关节角（度）。与 recordSample 共用时间轴，用于各轴角速度曲线。
   * @param {number} timestamp 秒
   * @param {number[]} jointsDeg 各关节角度（度）
   */
  recordJointSample(timestamp, jointsDeg) {
    if (!Array.isArray(jointsDeg) || !jointsDeg.length) return;
    const time = finiteOr(timestamp, this.jointSamples.length ? this.jointSamples.at(-1).t : 0);
    this.jointSamples.push({ t: time, joints: jointsDeg.map((value) => finiteOr(Number(value), 0)), tag: this.activeTag });
    const overflow = this.jointSamples.length - this.maxSamples;
    if (overflow > 200) this.jointSamples.splice(0, overflow);
  }

  /** 保存最近一次 MOVL 规划结果，供奇异区 DLS 阻尼曲线使用。 */
  setPlan(plan) {
    this.plan = plan && Array.isArray(plan.samples) && plan.samples.length ? plan : null;
    return this.plan;
  }

  /**
   * 按标签切分本次采样的 MOVJ / MOVL 段，并算出直线度、角速度、阻尼三组对比数据。
   * 没有数据时返回 null，不抛错。
   */
  buildComparison(options = {}) {
    const samples = this.samples;
    if (!samples.length) { this.comparison = null; return null; }
    const group = (list) => {
      const buckets = { MOVJ: [], MOVL: [] };
      for (const entry of list) (buckets[entry.tag === 'MOVL' ? 'MOVL' : 'MOVJ']).push(entry);
      return buckets;
    };
    const pointsByTag = group(samples);
    const jointsByTag = group(this.jointSamples);
    const usable = (list) => (list.length >= 2 ? list : null);

    const straightness = {
      MOVJ: usable(pointsByTag.MOVJ) ? straightnessDeviation(pointsByTag.MOVJ) : null,
      MOVL: usable(pointsByTag.MOVL) ? straightnessDeviation(pointsByTag.MOVL) : null,
    };
    const velocities = {
      MOVJ: usable(jointsByTag.MOVJ) ? jointVelocities(jointsByTag.MOVJ) : null,
      MOVL: usable(jointsByTag.MOVL) ? jointVelocities(jointsByTag.MOVL) : null,
    };
    const damping = dampingProfile(this.plan);
    const comparison = {
      builtAt: samples.at(-1).t,
      counts: { samples: samples.length, joints: this.jointSamples.length, plan: this.plan?.samples?.length ?? 0 },
      points: { MOVJ: pointsByTag.MOVJ, MOVL: pointsByTag.MOVL },
      straightness, velocities, damping,
      summary: compareTrajectories({ straightness, velocities, plan: this.plan }),
      options,
    };
    this.comparison = comparison;
    return comparison;
  }

  /** 对比数据导出为 CSV（三张图各一段，便于写报告）。 */
  exportComparisonCSV() {
    const comparison = this.comparison ?? this.buildComparison();
    if (!comparison) return '';
    const rows = ['section,series,t,s/key,value'];
    for (const tag of ['MOVJ', 'MOVL']) {
      const data = comparison.straightness[tag];
      if (!data) continue;
      for (const sample of data.samples) {
        rows.push(`straightness,${tag},${sample.t.toFixed(4)},${sample.s.toFixed(6)},${sample.distance.toFixed(6)}`);
      }
    }
    for (const tag of ['MOVJ', 'MOVL']) {
      const data = comparison.velocities[tag];
      if (!data) continue;
      data.series.forEach((joint, index) => {
        for (const point of joint.values) {
          rows.push(`jointVelocity,${tag}-${joint.name},${point.t.toFixed(4)},${index},${point.v.toFixed(6)}`);
        }
      });
    }
    for (const sample of comparison.damping?.samples ?? []) {
      rows.push(`damping,,${sample.s.toFixed(6)},${sample.lambda.toFixed(6)},${sample.manipulability.toFixed(6)}`);
    }
    return rows.join('\n');
  }

  /** 对比统计表（报告 5.3–5.5 的表格数据）。 */
  getComparisonStats() {
    const comparison = this.comparison ?? this.buildComparison();
    return comparison?.summary ?? [];
  }

  /** 渲染 MOVJ vs MOVL 对比视图：直线度偏差 / 各轴角速度 / 奇异区 DLS 阻尼。 */
  renderComparison(container) {
    if (!container) return;
    this.compareContainer = container;
    this.theme = pickTheme(container);
    let canvas = container.querySelector('canvas.trajectory-compare-canvas');
    if (!canvas) {
      canvas = document.createElement('canvas');
      canvas.className = 'trajectory-compare-canvas';
      canvas.setAttribute('role', 'img');
      canvas.setAttribute('aria-label', 'MOVJ 与 MOVL 直线度偏差、各轴角速度、奇异区 DLS 阻尼曲线');
      canvas.style.display = 'block';
      canvas.style.width = '100%';
      canvas.style.height = `${COMPARE_HEIGHT}px`;
      container.appendChild(canvas);
    }
    this.compareCanvas = canvas;
    this.compareCtx = canvas.getContext('2d');
    this.drawComparison();
  }

  /** 直接绘制对比视图（renderComparison 之后内部调用，也可手动调）。 */
  drawComparison() {
    const canvas = this.compareCanvas;
    const ctx = this.compareCtx;
    if (!canvas || !ctx) return;
    const comparison = this.comparison ?? this.buildComparison();
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth || 860;
    const height = canvas.clientHeight || COMPARE_HEIGHT;
    const pixelWidth = Math.max(1, Math.round(width * dpr));
    const pixelHeight = Math.max(1, Math.round(height * dpr));
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = this.theme.background;
    ctx.fillRect(0, 0, width, height);

    const pad = 10, gap = 10;
    const panelHeight = Math.floor((height - pad * 2 - gap * 2) / 3);
    const rects = [0, 1, 2].map((index) => ({ x: pad, y: pad + index * (panelHeight + gap), w: width - pad * 2, h: panelHeight }));

    if (!comparison) {
      for (const rect of rects) this._panel(ctx, rect, { title: '对比数据', empty: '暂无数据：先录一段 MOVJ，再录一段 MOVL' });
      return;
    }

    // 面板 1：直线度偏差（点到起终点弦线的垂距 / 归一化行程）
    const straightnessSeries = [];
    for (const tag of ['MOVJ', 'MOVL']) {
      const data = comparison.straightness[tag];
      if (!data) continue;
      straightnessSeries.push({
        name: tag,
        color: tag === 'MOVJ' ? this.theme.speed : this.theme.y,
        points: data.samples.map((sample) => [sample.s, sample.distance]),
      });
    }
    const maxDeviation = Math.max(
      1e-6,
      ...['MOVJ', 'MOVL'].map((tag) => comparison.straightness[tag]?.max ?? 0),
    );
    this._panel(ctx, rects[0], {
      title: `直线度偏差 · 到起终点弦线的垂距（${this.unitLabel}）`,
      xTicks: niceTicks(0, 1, 5),
      yTicks: niceTicks(0, maxDeviation * 1.15, 4),
      xFormat: (value) => value.toFixed(2),
      yFormat: (value) => (value >= 0.01 ? value.toFixed(2) : value.toExponential(1)),
      series: straightnessSeries,
      empty: straightnessSeries.length ? null : '缺少轨迹段',
      axisHint: '横轴：归一化行程',
    });

    // 面板 2：各轴角速度
    const velocitySeries = [];
    const jointCount = Math.max(comparison.velocities.MOVJ?.series.length ?? 0, comparison.velocities.MOVL?.series.length ?? 0);
    for (let index = 0; index < jointCount; index += 1) {
      const data = comparison.velocities.MOVL?.series[index] ?? comparison.velocities.MOVJ?.series[index];
      if (!data) continue;
      velocitySeries.push({
        name: data.name,
        color: JOINT_COLORS[index % JOINT_COLORS.length],
        points: data.values.map((point) => [point.t, point.v]),
      });
    }
    const maxVelocity = Math.max(1e-6, ...velocitySeries.flatMap((series) => series.points.map((point) => Math.abs(point[1]))));
    const timeRange = timeSpanOf(comparison.velocities);
    this._panel(ctx, rects[1], {
      title: `各轴角速度 · ${comparison.velocities.MOVL ? 'MOVL' : 'MOVJ'} 段（度/秒）`,
      xTicks: niceTicks(timeRange[0], timeRange[1], 5),
      yTicks: niceTicks(0, maxVelocity * 1.1, 4),
      xFormat: (value) => value.toFixed(1),
      yFormat: (value) => value.toFixed(0),
      series: velocitySeries,
      empty: velocitySeries.length ? null : '缺少关节角采样',
      axisHint: '横轴：时间 (s)',
    });

    // 面板 3：奇异区 DLS 阻尼曲线
    const damping = comparison.damping;
    if (!damping || !damping.samples.length) {
      this._panel(ctx, rects[2], { title: '奇异区 DLS 阻尼 λ(s) 与可操作度', empty: '尚无 MOVL 规划结果' });
    } else {
      const maxLambda = Math.max(1e-6, damping.maxLambda);
      this._panel(ctx, rects[2], {
        title: `奇异区 DLS 阻尼 λ(s) · 奇异点 ${damping.singularCount} 个`,
        xTicks: niceTicks(0, 1, 5),
        yTicks: niceTicks(0, maxLambda * 1.2, 4),
        xFormat: (value) => value.toFixed(2),
        yFormat: (value) => value.toFixed(3),
        series: [{ name: 'λ', color: this.theme.x, fill: true, points: damping.samples.map((sample) => [sample.s, sample.lambda]) }],
        bands: damping.singularRanges.map((range) => ({ from: range.sFrom, to: range.sTo, color: 'rgba(255, 138, 138, 0.18)' })),
        axisHint: '横轴：归一化行程（红色带 = 奇异位形）',
      });
    }
  }

  /** 导出采样数据为 CSV 文本（保持骨架约定的表头与精度）
   * @returns {string}
   */
  exportToCSV() {
    const header = 't,x,y,z,v';
    const rows = this.samples.map(
      (sample) => `${sample.t.toFixed(4)},${sample.x.toFixed(6)},${sample.y.toFixed(6)},${sample.z.toFixed(6)},${sample.v.toFixed(6)}`,
    );
    return [header, ...rows].join('\n');
  }

  /** 浏览器端下载 CSV（方便写报告时直接拿数据） */
  downloadCSV(filename = 'trajectory.csv') {
    const blob = new Blob(['\ufeff' + this.exportToCSV()], { type: 'text/csv;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
    return this.samples.length;
  }
}

/* ------------------------------------------------------------------ *
 * 【王静 · 组员 D】实验二增量的纯计算部分
 *
 * 下面四个函数不依赖 DOM、不 import three，可直接在 Node 里单元测试；
 * 界面只负责把它们的返回值画出来。长度单位一律沿用资产单位，角度单位是度。
 * ------------------------------------------------------------------ */

/** 把 {x,y,z} / [x,y,z] 统一成点对象；非法输入返回 null。 */
function toPoint(value) {
  if (Array.isArray(value) && value.length >= 3) {
    const [x, y, z] = value;
    return Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z) ? { x, y, z } : null;
  }
  if (value && typeof value === 'object') {
    const { x, y, z, t } = value;
    if (![x, y, z].every(Number.isFinite)) return null;
    return Number.isFinite(t) ? { x, y, z, t } : { x, y, z };
  }
  return null;
}

const subPoint = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const dotPoint = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const lengthOf = (a) => Math.hypot(a.x, a.y, a.z);
const distanceBetween = (a, b) => lengthOf(subPoint(a, b));

/**
 * 直线度偏差：每个采样点到「起点—终点」弦线的垂距。
 * 理想直线运动时全部为 0；MOVJ 走的是关节空间圆弧，垂距会明显大于 0。
 *
 * @param {Array<{x:number,y:number,z:number,t?:number}>} points
 * @param {{x,y,z}} [start] 缺省取首点
 * @param {{x,y,z}} [end] 缺省取末点
 */
export function straightnessDeviation(points, start = null, end = null) {
  const list = (points ?? []).map(toPoint).filter(Boolean);
  if (list.length < 1) {
    return { count: 0, max: 0, mean: 0, rms: 0, pathLength: 0, chord: 0, percent: 0, samples: [], start: null, end: null };
  }
  const from = toPoint(start) ?? list[0];
  const to = toPoint(end) ?? list.at(-1);
  const chordVector = subPoint(to, from);
  const chord = lengthOf(chordVector);
  const direction = chord > 1e-12 ? { x: chordVector.x / chord, y: chordVector.y / chord, z: chordVector.z / chord } : { x: 0, y: 0, z: 0 };

  const samples = [];
  let max = 0, sum = 0, sumSquares = 0, pathLength = 0;
  for (let index = 0; index < list.length; index += 1) {
    const point = list[index];
    const relative = subPoint(point, from);
    const along = dotPoint(relative, direction);
    const deviation = Math.hypot(
      relative.x - direction.x * along,
      relative.y - direction.y * along,
      relative.z - direction.z * along,
    );
    samples.push({
      index,
      t: Number.isFinite(point.t) ? point.t : index,
      s: chord > 1e-12 ? along / chord : 0,
      along,
      distance: deviation,
    });
    max = Math.max(max, deviation);
    sum += deviation;
    sumSquares += deviation * deviation;
    if (index > 0) pathLength += distanceBetween(list[index - 1], point);
  }
  return {
    count: list.length,
    max,
    mean: sum / list.length,
    rms: Math.sqrt(sumSquares / list.length),
    pathLength,
    chord,
    percent: chord > 1e-12 ? (max / chord) * 100 : 0,
    detour: chord > 1e-12 ? pathLength / chord : 1,
    samples,
    start: from,
    end: to,
  };
}

/**
 * 各轴角速度：相邻采样帧的关节角差商（度/秒）。
 * 采样断档（dt 过小或为负）时记 0，避免把帧抖动放大成尖峰。
 *
 * @param {Array<{t:number,joints:number[]}>} frames
 * @param {{names?:string[], minDt?:number}} [options]
 */
export function jointVelocities(frames, options = {}) {
  const clean = (frames ?? [])
    .filter((frame) => Array.isArray(frame?.joints) && frame.joints.length && Number.isFinite(frame.t))
    .map((frame) => ({ t: frame.t, joints: frame.joints.map(Number) }));
  if (clean.length < 2) return { count: clean.length, series: [], max: 0, meanAbs: 0, span: [0, 0] };
  const jointCount = Math.max(...clean.map((frame) => frame.joints.length));
  const names = Array.isArray(options.names) && options.names.length >= jointCount
    ? options.names
    : Array.from({ length: jointCount }, (_, index) => `J${index + 1}`);
  const minDt = Number.isFinite(options.minDt) ? options.minDt : 1e-6;
  const t0 = clean[0].t;

  const series = [];
  let max = 0, sumAbs = 0, count = 0;
  for (let joint = 0; joint < jointCount; joint += 1) {
    const values = [];
    let jointMax = 0;
    for (let index = 1; index < clean.length; index += 1) {
      const dt = clean[index].t - clean[index - 1].t;
      const speed = dt > minDt
        ? (clean[index].joints[joint] - clean[index - 1].joints[joint]) / dt
        : 0;
      values.push({ t: clean[index].t - t0, v: speed });
      jointMax = Math.max(jointMax, Math.abs(speed));
      sumAbs += Math.abs(speed);
      count += 1;
    }
    series.push({ name: names[joint], values, max: jointMax });
    max = Math.max(max, jointMax);
  }
  return {
    count: clean.length,
    series,
    max,
    meanAbs: count ? sumAbs / count : 0,
    span: [0, clean.at(-1).t - t0],
    maxJoint: series.reduce((best, item) => (item.max > (best?.max ?? -1) ? item : best), null)?.name ?? '',
  };
}

/**
 * 奇异区 DLS 阻尼曲线：把 MOVL 规划结果里的 dampingUsed / manipulability / singular
 * 整理成「归一化行程 s → λ」的序列，并合并连续的奇异标记区间。
 *
 * @param {{samples:Array<{t?:number,dampingUsed?:number|null,manipulability?:number|null,singular?:boolean}>}|null} plan
 * @param {{gap?:number}} [options]
 */
export function dampingProfile(plan, options = {}) {
  const samples = Array.isArray(plan?.samples) ? plan.samples : [];
  if (!samples.length) {
    return { samples: [], maxLambda: 0, meanLambda: 0, singularCount: 0, singularRanges: [], minManipulability: null };
  }
  const total = samples.length > 1 ? samples.length - 1 : 1;
  const gap = Number.isFinite(options.gap) ? Math.max(0, options.gap) : 1;
  const points = [];
  let maxLambda = 0, sumLambda = 0, singularCount = 0;
  let minManipulability = null;

  for (let index = 0; index < samples.length; index += 1) {
    const sample = samples[index] ?? {};
    const s = Number.isFinite(sample.t) ? sample.t : index / total;
    const lambda = Number.isFinite(sample.dampingUsed) ? sample.dampingUsed : 0;
    const manipulability = Number.isFinite(sample.manipulability) ? sample.manipulability : 0;
    const singular = sample.singular === true;
    points.push({ index, s, lambda, manipulability, singular });
    maxLambda = Math.max(maxLambda, lambda);
    sumLambda += lambda;
    if (singular) singularCount += 1;
    if (minManipulability === null || manipulability < minManipulability) minManipulability = manipulability;
  }

  // 合并连续的奇异点（允许中间夹少量非奇异点，视为同一段奇异区）
  const singularRanges = [];
  let current = null;
  for (const point of points) {
    if (point.singular) {
      if (!current) current = { from: point.index, to: point.index, sFrom: point.s, sTo: point.s, count: 0, gapRun: 0 };
      current.to = point.index; current.sTo = point.s; current.count += 1; current.gapRun = 0;
    } else if (current) {
      current.gapRun += 1;
      if (current.gapRun > gap) {
        singularRanges.push(current);
        current = null;
      }
    }
  }
  if (current) singularRanges.push(current);

  return {
    samples: points,
    maxLambda,
    meanLambda: sumLambda / points.length,
    singularCount,
    singularRanges,
    minManipulability,
  };
}

/** 轨迹对比的汇总表（报告 5.3–5.5 的表格直接取这里的数值）。 */
export function compareTrajectories({ straightness, velocities, plan } = {}) {
  const rows = [];
  const push = (metric, unit, pick) => {
    rows.push({ metric, unit, MOVJ: pick(straightness?.MOVJ, velocities?.MOVJ), MOVL: pick(straightness?.MOVL, velocities?.MOVL) });
  };
  push('最大直线度偏差', '资产单位', (s) => (s ? s.max : null));
  push('偏差 / 弦长', '%', (s) => (s ? s.percent : null));
  push('轨迹长度', '资产单位', (s) => (s ? s.pathLength : null));
  push('迂回系数（路径 / 弦长）', '—', (s) => (s ? s.detour : null));
  push('最大关节角速度', '度/秒', (_, v) => (v ? v.max : null));
  push('平均关节角速度', '度/秒', (_, v) => (v ? v.meanAbs : null));
  rows.push({ metric: '采样帧数', unit: '帧', MOVJ: velocities?.MOVJ?.count ?? 0, MOVL: velocities?.MOVL?.count ?? 0 });
  if (plan) {
    rows.push({
      metric: 'MOVL 规划质量',
      unit: '—',
      MOVJ: null,
      MOVL: `插补点 ${plan.sampleCount ?? plan.samples?.length ?? 0} · 最大位置误差 ${Number.isFinite(plan.maxPosError) ? plan.maxPosError.toExponential(2) : '—'} · 奇异点 ${plan.singularCount ?? 0}`,
    });
  }
  return rows;
}

/** 时间轴范围（对比视图的横轴）。 */
function timeSpanOf(velocities) {
  let min = Infinity, max = -Infinity;
  for (const tag of ['MOVJ', 'MOVL']) {
    for (const series of velocities?.[tag]?.series ?? []) {
      for (const point of series.values) {
        if (point.t < min) min = point.t;
        if (point.t > max) max = point.t;
      }
    }
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [0, 1];
  return max > min ? [min, max] : [min, min + 1];
}


