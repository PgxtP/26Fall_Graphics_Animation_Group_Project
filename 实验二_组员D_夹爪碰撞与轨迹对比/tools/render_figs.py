"""第 5 章图表渲染（Pillow，无 matplotlib）：读取 out/chapter5/data.json 生成 5 张 PNG。"""
import json
import math
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', '第五章_图', 'data.json')
OUT = os.path.join(HERE, '..', '第五章_图')
FONT = r'C:\Windows\Fonts\msyh.ttc'
FONT_BOLD = r'C:\Windows\Fonts\msyhbd.ttc'

W, H = 1120, 660
M = dict(left=110, right=50, top=104, bottom=150)
COLORS = ['#2f7fd8', '#0f9c85', '#c8811a', '#d24f7f', '#7a5cd6', '#5a8f2f']
INK = '#1f3b57'
MUTED = '#5b6b7c'
GRID = '#e2e8ef'


def font(size, bold=False):
    path = FONT_BOLD if bold and os.path.exists(FONT_BOLD) else FONT
    return ImageFont.truetype(path, size)


def make_canvas(title, subtitle=None):
    image = Image.new('RGB', (W, H), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((M['left'], 26), title, fill=INK, font=font(25, True))
    if subtitle:
        draw.text((M['left'], 62), subtitle, fill=MUTED, font=font(13))
    return image, draw


def plot_area():
    return (M['left'], M['top'], W - M['right'], H - M['bottom'])


def fmt(value, digits=4):
    if abs(value) < 1e-12:
        return '0'
    if abs(value) < 1e-5:
        return f'{value:.1e}'
    if abs(value) >= 1:
        return f'{value:.{digits}f}'.rstrip('0').rstrip('.')
    return f'{value:.{digits}f}'.rstrip('0').rstrip('.')


def nice_step(span, count):
    raw = span / max(2, count)
    magnitude = 10 ** math.floor(math.log10(raw))
    norm = raw / magnitude
    factor = 1 if norm <= 1 else 2 if norm <= 2 else 2.5 if norm <= 2.5 else 5 if norm <= 5 else 10
    return factor * magnitude


def tick_range(values, non_negative=True, count=5):
    lo, hi = min(values), max(values)
    if non_negative:
        lo = min(lo, 0.0)
    if hi - lo < 1e-12:
        hi = lo + 1.0
    step = nice_step(hi - lo, count)
    start = math.floor(lo / step) * step
    end = math.ceil(hi / step) * step
    if non_negative and start < 0 <= lo:
        start = 0.0
    return start, end, step


def vertical_text(image, text, position, text_font, fill):
    probe = Image.new('RGBA', (420, 34), (0, 0, 0, 0))
    ImageDraw.Draw(probe).text((0, 0), text, font=text_font, fill=fill)
    rotated = probe.rotate(90, expand=True)
    image.paste(rotated, position, rotated)


def axes(image, draw, xr, yr, xlabel, ylabel, xticks=5, yticks=5, xstep=None, ystep=None):
    x0, y0, x1, y1 = plot_area()
    x_values = []
    if xstep:
        value = math.ceil(xr[0] / xstep - 1e-9) * xstep
        while value <= xr[1] + 1e-9:
            x_values.append(value)
            value += xstep
    else:
        x_values = [xr[0] + (xr[1] - xr[0]) * i / xticks for i in range(xticks + 1)]
    y_values = []
    if ystep:
        value = math.ceil(yr[0] / ystep - 1e-9) * ystep
        while value <= yr[1] + 1e-9:
            y_values.append(value)
            value += ystep
    else:
        y_values = [yr[0] + (yr[1] - yr[0]) * i / yticks for i in range(yticks + 1)]
    for value in x_values:
        x = x0 + (value - xr[0]) / (xr[1] - xr[0]) * (x1 - x0)
        draw.line([(x, y0), (x, y1)], fill=GRID, width=1)
        text = fmt(value)
        draw.text((x - len(text) * 3.4, y1 + 10), text, fill=MUTED, font=font(12))
    for value in y_values:
        y = y1 - (value - yr[0]) / (yr[1] - yr[0]) * (y1 - y0)
        draw.line([(x0, y), (x1, y)], fill=GRID, width=1)
        text = fmt(value)
        draw.text((x0 - 12 - len(text) * 6.4, y - 8), text, fill=MUTED, font=font(12))
    draw.rectangle([x0, y0, x1, y1], outline='#b9c6d4', width=1)
    label_width = draw.textlength(xlabel, font=font(14))
    draw.text(((x0 + x1) / 2 - label_width / 2, y1 + 40), xlabel, fill=INK, font=font(14))
    vertical_text(image, ylabel, (x0 - 92, y0 - 30), font(14), INK)


def sub_map(box, xr, yr, x, y):
    bx0, by0, bx1, by1 = box
    return (bx0 + (x - xr[0]) / (xr[1] - xr[0]) * (bx1 - bx0),
            by1 - (y - yr[0]) / (yr[1] - yr[0]) * (by1 - by0))


def map_point(xr, yr, x, y):
    return sub_map(plot_area(), xr, yr, x, y)


def dashed(draw, p0, p1, color, width, dash=9):
    x0, y0 = p0
    x1, y1 = p1
    total = max(1e-9, ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5)
    step, on = 0.0, True
    while step < total:
        seg = min(dash, total - step)
        if on:
            draw.line([(x0 + (x1 - x0) * step / total, y0 + (y1 - y0) * step / total),
                       (x0 + (x1 - x0) * (step + seg) / total, y0 + (y1 - y0) * (step + seg) / total)],
                      fill=color, width=width)
        step += seg
        on = not on


def polyline(draw, xr, yr, points, color, width=2, dash=None, box=None):
    box = box or plot_area()
    pts = [sub_map(box, xr, yr, x, y) for x, y in points]
    if len(pts) < 2:
        return pts
    if dash:
        for i in range(len(pts) - 1):
            dashed(draw, pts[i], pts[i + 1], color, width, dash)
    else:
        draw.line(pts, fill=color, width=width, joint='curve')
    return pts


def legend_below(draw, entries, columns=1):
    x0, _, x1, y1 = plot_area()
    per_column = max(1, (len(entries) + columns - 1) // columns)
    for index, (name, color, width, dash) in enumerate(entries):
        column = index // per_column
        row = index % per_column
        px = x0 + column * ((x1 - x0) // columns + 10)
        py = y1 + 74 + row * 22
        if dash:
            dashed(draw, (px, py + 7), (px + 26, py + 7), color, width, dash)
        else:
            draw.line([(px, py + 7), (px + 26, py + 7)], fill=color, width=width)
        draw.text((px + 34, py - 3), name, fill=INK, font=font(13))


def save(image, name):
    path = os.path.join(OUT, name)
    image.save(path, 'PNG')
    print('wrote', os.path.basename(path))


def fig51(data):
    image, draw = make_canvas(
        '图 5-1  MOVJ 与 MOVL 直线度偏差对比',
        f"同一对起终点，弦长 {data['endpoints']['distance']:.3f} 资产单位；纵轴为采样点到弦线的垂距",
    )
    xr = (0.0, 1.0)
    values = [p['deviation'] for p in data['movj']['series']] + [p['deviation'] for p in data['movl']['series']]
    y_start, y_end, y_step = tick_range(values)
    yr = (y_start, y_end)
    axes(image, draw, xr, yr, '归一化行程 s', '垂距（资产单位）', xstep=0.2, ystep=y_step)
    polyline(draw, xr, yr, [(0, 0), (1, 0)], '#9aa9b8', 1)
    polyline(draw, xr, yr, [(p['s'], p['deviation']) for p in data['movj']['series']], COLORS[3], 2)
    polyline(draw, xr, yr, [(p['s'], p['deviation']) for p in data['movl']['series']], COLORS[1], 3)
    legend_below(draw, [
        (f"MOVJ：峰值 {data['movj']['straightness']['max']:.4f}（{data['movj']['straightness']['percent']:.2f}% 弦长）", COLORS[3], 2, None),
        (f"MOVL：峰值 {data['movl']['straightness']['max']:.4f}（{data['movl']['straightness']['percent']:.2f}% 弦长）", COLORS[1], 3, None),
        ('理想直线（偏差 0）', '#9aa9b8', 1, None),
    ], columns=3)
    save(image, '图5-1_直线度偏差.png')


def fig52(data):
    image, draw = make_canvas(
        '图 5-2  各轴角速度（MOVJ 实线 / MOVL 虚线）',
        '由相邻帧关节角差商得到；横轴为各自运动段的归一化进度，便于直接对比两条段的快慢',
    )
    movj = [s for s in data['velocitySeries']['MOVJ'] if any(abs(v) > 1e-9 for _, v in s['points'])]
    movl = [s for s in data['velocitySeries']['MOVL'] if any(abs(v) > 1e-9 for _, v in s['points'])]

    def normalized(series):
        times = [t for t, _ in series['points']]
        total = max(times) or 1.0
        return [(t / total, v) for t, v in series['points']]

    values = [v for s in movj + movl for _, v in s['points']]
    y_start, y_end, y_step = tick_range(values, non_negative=False)
    xi = (-0.02, 1.02)
    yr = (y_start, y_end)
    axes(image, draw, xi, yr, '归一化进度', '角速度（度/秒）', xstep=0.2, ystep=y_step)
    for index, series in enumerate(movj):
        polyline(draw, xi, yr, normalized(series), COLORS[index % len(COLORS)], 2)
    for index, series in enumerate(movl):
        polyline(draw, xi, yr, normalized(series), COLORS[index % len(COLORS)], 2, dash=8)
    entries = [(s['name'], COLORS[i % len(COLORS)], 2, None) for i, s in enumerate(movj)]
    entries.append(('实线 = MOVJ，虚线 = MOVL', MUTED, 1, None))
    legend_below(draw, entries, columns=3)
    save(image, '图5-2_各轴角速度.png')


def fig53(data):
    image, draw = make_canvas(
        '图 5-3  奇异区 DLS 自适应阻尼',
        '左：求解器阻尼律 λ(w)；右：同一条直线的 λ(s)，位置约束 3×5 与姿态约束 6×5 两种雅可比口径',
    )
    x0, y0, x1, y1 = plot_area()
    y1 = y1 + 40  # 双栏图不需要图例，向下借用一点空间
    middle = (x0 + x1) // 2
    left = (x0, y0, middle - 18, y1)
    right = (middle + 18, y0, x1, y1)

    law = data['damping']['law']
    lxr, lyr = (0.0, 0.004), (0.0, 0.17)
    curve = [(p['w'], p['lambda']) for p in law['points']] + [(lxr[1], law['min'])]
    pts = [sub_map(left, lxr, lyr, x, y) for x, y in curve]
    draw.line(pts, fill=COLORS[0], width=3)
    for point in data['damping']['fromPlan']['points'][1:]:
        px, py = sub_map(left, lxr, lyr, point[2], point[1])
        draw.rectangle([px - 4, py - 4, px + 4, py + 4], fill=COLORS[2])
    draw.rectangle(left, outline='#b9c6d4', width=1)
    draw.text((left[0] + 10, left[1] + 8), 'λ(w)：w 趋近 0 时阻尼抬升到上限', fill=INK, font=font(13, True))
    draw.text((left[0] + 10, left[1] + 34), f"阈值 w = {law['threshold']:.0e}，min = {law['min']}，max = {law['max']}", fill=MUTED, font=font(11))
    text_x = left[0] + (left[2] - left[0]) // 2 + 16
    draw.text((text_x, left[1] + 96), '橙方 = MOVL 带姿态样本', fill=COLORS[2], font=font(12))
    draw.text((text_x, left[1] + 118), '（w = 0，λ = 0.15）', fill=COLORS[2], font=font(12))
    draw.text((text_x, left[1] + 150), '压力测试实测 w ∈ [0.05, 0.15]', fill=COLORS[3], font=font(12))
    draw.text((text_x, left[1] + 172), '→ λ 恒为下限，落点在本图右侧之外', fill=COLORS[3], font=font(12))
    draw.text((left[0] + 10, left[3] - 8), '横轴 w（可操作度）· 纵轴 λ（阻尼）', fill=MUTED, font=font(11))

    rxr, ryr = (0.0, 1.0), (0.0, 0.17)
    for point in data['damping']['fromPlan']['points']:
        px, py = sub_map(right, rxr, ryr, point[0], point[1])
        draw.line([(px - 6, py), (px + 6, py)], fill=COLORS[2], width=3)
    for point in data['damping']['positionOnly']['points']:
        px, py = sub_map(right, rxr, ryr, point[0], point[1])
        draw.line([(px - 6, py), (px + 6, py)], fill=COLORS[1], width=3)
    draw.rectangle(right, outline='#b9c6d4', width=1)
    draw.text((right[0] + 10, right[1] + 8), 'λ(s)：两种雅可比口径', fill=INK, font=font(13, True))
    draw.text((right[0] + 14, right[1] + 78), '橙 = 6×5（秩亏，λ 恒 0.15）', fill=COLORS[2], font=font(12))
    draw.text((right[0] + 14, right[1] + 100), '绿 = 3×5（w ≈ 0.09~0.30，λ 恒 0.005）', fill=COLORS[1], font=font(12))
    draw.text((right[0] + 10, right[3] - 8), '横轴 s（归一化行程）· 纵轴 λ（阻尼）', fill=MUTED, font=font(11))
    save(image, '图5-3_DLS阻尼曲线.png')


def fig54(data):
    image, draw = make_canvas(
        '图 5-4  MOVJ 与 MOVL 的末端轨迹（Y–Z 平面投影）',
        '该段运动 X 恒定，故取 Y–Z 投影；MOVJ 沿关节空间插值走出弧线，MOVL 贴合起终点连线',
    )
    start = data['endpoints']['start']
    span = max(abs(data['endpoints']['end'][1] - start[1]), abs(data['endpoints']['end'][2] - start[2]), 1e-9)

    def norm(point):
        return ((point[1] - start[1]) / span, (point[2] - start[2]) / span)

    movj = [norm(p['position']) for p in data['movj']['series']]
    movl = [norm(p['position']) for p in data['movl']['series']]
    values = [v for p in movj + movl for v in p]
    x_start, x_end, x_step = tick_range(values, non_negative=False, count=4)
    y_start, y_end, y_step = tick_range(values, non_negative=False, count=4)
    xr, yr = (x_start, x_end), (y_start, y_end)
    axes(image, draw, xr, yr, '归一化 Y', '归一化 Z', xstep=x_step, ystep=y_step)
    polyline(draw, xr, yr, [movj[0], movl[-1]], '#9aa9b8', 2, dash=6)
    polyline(draw, xr, yr, movj, COLORS[3], 2)
    polyline(draw, xr, yr, movl, COLORS[1], 3)
    for point in (movj[0], movl[-1]):
        px, py = map_point(xr, yr, *point)
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=INK)
    legend_below(draw, [
        ('MOVJ 实际轨迹', COLORS[3], 2, None),
        ('MOVL 实际轨迹', COLORS[1], 3, None),
        ('起终点连线（理想直线）', '#9aa9b8', 2, 6),
    ], columns=3)
    save(image, '图5-4_轨迹对比.png')


def fig55(data):
    image, draw = make_canvas(
        '图 5-5  夹爪接近台面时的最小间距',
        '间距由 checkGripperCollision 的「顶点—三角面片」最近距离给出；归零即发生穿透',
    )
    clearance = data['gripper']['clearance']
    xr = (min(c['approach'] for c in clearance), max(c['approach'] for c in clearance))
    gaps = [c['gap'] for c in clearance]
    y_start, y_end, y_step = tick_range(gaps)
    yr = (y_start, y_end)
    axes(image, draw, xr, yr, '接近位移（资产单位）', '最小间距（资产单位）', xstep=0.05, ystep=y_step)
    polyline(draw, xr, yr, [(c['approach'], c['gap']) for c in clearance], COLORS[0], 3)
    contact = data['gripper']['firstContact']
    if contact:
        px, py = map_point(xr, yr, contact['approach'], 0)
        draw.ellipse([px - 7, py - 7, px + 7, py + 7], outline=COLORS[3], width=3)
        draw.text((px - 54, py - 36), f"首次穿透 {contact['approach']:.2f}", fill=COLORS[3], font=font(13))
    entries = [('最小间距（margin = 0：只拦穿透）', COLORS[0], 3, None)]
    entries += [(f"margin = {e['margin']:.3f} → {'拦截' if e['colliding'] else '放行'}", MUTED, 1, None)
                for e in data['gripper']['marginSweep']]
    legend_below(draw, entries, columns=2)
    save(image, '图5-5_夹爪最小间距.png')


def main():
    with open(DATA, encoding='utf-8') as handle:
        data = json.load(handle)
    fig51(data)
    fig52(data)
    fig53(data)
    fig54(data)
    fig55(data)


if __name__ == '__main__':
    main()
