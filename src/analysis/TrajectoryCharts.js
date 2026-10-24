/**
 * 【5 号位 · 组员 D】TrajectoryCharts —— 末端轨迹记录、图表绘制与 CSV 导出
 *
 * 对外接口（与仓库骨架一致，未改动任何签名）：
 *   const charts = new TrajectoryCharts();
 *   charts.recordSample(timestamp, tcpPosition, linearVelocity);  // 动画循环里高频调用
 *   charts.renderCharts(chartContainer);                          // 传入一个空的 DOM 容器
 *   charts.exportToCSV();                                         // 返回 CSV 字符串
 *   charts.getStats();                                            // 额外提供：最大/平均速度、轨迹长度
 *
 * 集成方式（组长在 main.js 里只需要三行）：
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

    this.samples.push({ t: time, x, y, z, v: Math.abs(speed) });

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
      this._panel(ctx, rectPath, { title: '末端轨迹 · XY 俯视', empty: '暂无数据' });
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
    let minY = Infinity;
    let maxY = -Infinity;

    for (const point of visible) {
      minPosition = Math.min(minPosition, point.x, point.y, point.z);
      maxPosition = Math.max(maxPosition, point.x, point.y, point.z);
      maxSpeed = Math.max(maxSpeed, point.v);
      minX = Math.min(minX, point.x);
      maxX = Math.max(maxX, point.x);
      minY = Math.min(minY, point.y);
      maxY = Math.max(maxY, point.y);
    }

    const padPosition = Math.max(0.002, (maxPosition - minPosition) * 0.08);
    const positionTicks = niceTicks(minPosition - padPosition, maxPosition + padPosition, 5);
    const speedTicks = niceTicks(0, Math.max(maxSpeed * 1.12, 0.01), 4);
    const timeTicks = niceTicks(tStart, Math.max(tEnd, tStart + 0.5), 5);

    const spanX = Math.max(maxX - minX, 0.02);
    const spanY = Math.max(maxY - minY, 0.02);
    const span = Math.max(spanX, spanY) * 1.2;
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;

    this._panel(ctx, rectPath, {
      title: '末端轨迹 · XY 俯视',
      xTicks: niceTicks(centerX - span / 2, centerX + span / 2, 4),
      yTicks: niceTicks(centerY - span / 2, centerY + span / 2, 4),
      xFormat: (value) => value.toFixed(2),
      yFormat: (value) => value.toFixed(2),
      series: [{ name: '轨迹', color: this.theme.path, points: visible.map((point) => [point.x, point.y]) }],
      markers: [
        { color: this.theme.start, point: [visible[0].x, visible[0].y] },
        { color: this.theme.end, point: [last.x, last.y] },
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

  /**
   * 导出采样数据为 CSV 文本（保持骨架约定的表头与精度）
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
