import assert from 'node:assert/strict';
import {
  TrajectoryCharts,
  straightnessDeviation,
  jointVelocities,
  dampingProfile,
  compareTrajectories,
} from '../analysis/TrajectoryCharts.js';

const near = (actual, expected, tolerance = 1e-9) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `期望 ${expected}，实际 ${actual}`);

// 1. 直线度：理想直线偏差为 0
{
  const points = [
    { t: 0, x: 0, y: 0, z: 0 },
    { t: 0.1, x: 1, y: 0, z: 0 },
    { t: 0.2, x: 2, y: 0, z: 0 },
  ];
  const result = straightnessDeviation(points);
  near(result.max, 0);
  near(result.mean, 0);
  near(result.chord, 2);
  near(result.pathLength, 2);
  near(result.detour, 1);
  near(result.percent, 0);
  assert.equal(result.count, 3);
  near(result.samples[0].s, 0);
  near(result.samples.at(-1).s, 1);
}

// 2. 直线度：三角形绕行，最大偏差 1、均值 1/4、RMS 1/2
{
  const result = straightnessDeviation([[0, 0, 0], [1, 0, 0], [1, 1, 0], [2, 0, 0]]);
  near(result.max, 1);
  near(result.mean, 1 / 4);
  near(result.rms, 0.5);
  near(result.chord, 2);
  near(result.pathLength, 1 + 1 + Math.SQRT2);
  near(result.percent, 50);
  near(result.samples[1].s, 0.5);
  near(result.samples[2].s, 0.5);
}

// 3. 直线度：显式起终点与空输入
{
  const result = straightnessDeviation([{ x: 0, y: 0, z: 0 }, { x: 0, y: 2, z: 0 }], { x: 0, y: 0, z: 0 }, { x: 4, y: 0, z: 0 });
  near(result.chord, 4);
  near(result.samples[1].distance, 2); // 点 (0,2,0) 到 x 轴的距离
  const empty = straightnessDeviation([]);
  assert.equal(empty.count, 0);
  assert.equal(empty.max, 0);
  assert.deepEqual(empty.samples, []);
}

// 4. 角速度：恒定 10 度/秒
{
  const frames = [0, 0.1, 0.2, 0.3].map((t, index) => ({ t, joints: [index, index * 2] }));
  const result = jointVelocities(frames);
  assert.equal(result.series.length, 2);
  assert.deepEqual(result.series.map((item) => item.name), ['J1', 'J2']);
  near(result.series[0].max, 10);
  near(result.series[1].max, 20);
  near(result.max, 20);
  near(result.meanAbs, 15);
  near(result.span[1], 0.3);
  assert.equal(result.maxJoint, 'J2');
}

// 5. 角速度：采样断档不制造尖峰
{
  const frames = [{ t: 0, joints: [0] }, { t: 0, joints: [5] }, { t: 0.1, joints: [6] }];
  const result = jointVelocities(frames);
  near(result.series[0].values[0].v, 0, 1e-12);
  near(result.series[0].values[1].v, 10);
  assert.equal(jointVelocities([{ t: 0, joints: [0] }]).series.length, 0);
}

// 6. 阻尼曲线：奇异区合并与统计
{
  const samples = Array.from({ length: 20 }, (_, index) => ({
    t: index / 19,
    dampingUsed: index >= 8 && index <= 12 ? 0.15 : 0.005,
    manipulability: index >= 8 && index <= 12 ? 0.0004 : 0.02,
    singular: index >= 8 && index <= 12,
  }));
  const result = dampingProfile({ samples });
  near(result.maxLambda, 0.15);
  assert.equal(result.singularCount, 5);
  assert.equal(result.singularRanges.length, 1);
  assert.equal(result.singularRanges[0].from, 8);
  assert.equal(result.singularRanges[0].to, 12);
  near(result.singularRanges[0].sFrom, 8 / 19);
  near(result.minManipulability, 0.0004);
  assert.equal(result.samples.length, 20);

  // 间隔 1 个非奇异点时默认合并；gap=0 时拆成两段
  const gapped = Array.from({ length: 10 }, (_, index) => ({ t: index / 9, dampingUsed: 0.1, manipulability: 0.01, singular: index === 3 || index === 5 }));
  assert.equal(dampingProfile({ samples: gapped }).singularRanges.length, 1);
  assert.equal(dampingProfile({ samples: gapped }, { gap: 0 }).singularRanges.length, 2);
  assert.deepEqual(dampingProfile(null).samples, []);
}

// 7. 对比汇总表
{
  const straightness = {
    MOVJ: straightnessDeviation([[0, 0, 0], [1, 1, 0], [2, 0, 0]]),
    MOVL: straightnessDeviation([[0, 0, 0], [1, 0, 0], [2, 0, 0]]),
  };
  const velocities = {
    MOVJ: jointVelocities([0, 0.1, 0.2].map((t, index) => ({ t, joints: [index, index] }))),
    MOVL: jointVelocities([0, 0.2, 0.4].map((t, index) => ({ t, joints: [index, index * 2] }))),
  };
  const rows = compareTrajectories({ straightness, velocities, plan: { sampleCount: 24, maxPosError: 1e-6, singularCount: 2 } });
  const metric = (name) => rows.find((row) => row.metric === name);
  near(metric('最大直线度偏差').MOVL, 0);
  near(metric('最大直线度偏差').MOVJ, 1);
  near(metric('偏差 / 弦长').MOVJ, 50);
  near(metric('迂回系数（路径 / 弦长）').MOVL, 1);
  near(metric('最大关节角速度').MOVJ, 10);
  assert.match(metric('MOVL 规划质量').MOVL, /插补点 24/);
  assert.match(metric('MOVL 规划质量').MOVL, /奇异点 2/);
}

// 8. 类集成：按标签切分 MOVJ / MOVL 并生成对比数据（不涉及 DOM）
{
  const charts = new TrajectoryCharts({ unitLabel: '资产单位' });
  charts.setTag('MOVJ');
  for (let index = 0; index <= 10; index += 1) {
    const t = index * 0.1;
    charts.recordSample(t, { x: index * 0.1, y: Math.sin((index / 10) * Math.PI) * 0.5, z: 0 }, 1);
    charts.recordJointSample(t, [index * 0.5, 0, 0, 0, 0]);
  }
  charts.setTag('MOVL');
  for (let index = 0; index <= 10; index += 1) {
    const t = 1.2 + index * 0.1;
    charts.recordSample(t, { x: index * 0.1, y: 0, z: 0 }, 1);
    charts.recordJointSample(t, [index * 0.5, 0, 0, 0, 0]);
  }
  charts.setPlan({
    sampleCount: 11,
    maxPosError: 2e-6,
    singularCount: 1,
    samples: Array.from({ length: 11 }, (_, index) => ({
      t: index / 10,
      dampingUsed: index === 5 ? 0.12 : 0.005,
      manipulability: index === 5 ? 0.0008 : 0.02,
      singular: index === 5,
    })),
  });

  const comparison = charts.buildComparison();
  assert.equal(comparison.counts.samples, 22);
  assert.equal(comparison.counts.joints, 22);
  near(comparison.straightness.MOVL.max, 0, 1e-9);
  assert.ok(comparison.straightness.MOVJ.max > 0.1, 'MOVJ 段应有明显偏差');
  assert.equal(comparison.damping.singularRanges.length, 1);
  assert.equal(charts.getComparisonStats().length, 8);
  assert.equal(charts.getComparisonStats()[0].metric, '最大直线度偏差');

  const csv = charts.exportComparisonCSV();
  const lines = csv.split('\n');
  assert.equal(lines[0], 'section,series,t,s/key,value');
  assert.ok(lines.some((line) => line.startsWith('straightness,MOVJ')));
  assert.ok(lines.some((line) => line.startsWith('straightness,MOVL')));
  assert.ok(lines.some((line) => line.startsWith('jointVelocity,MOVL-J1')));
  assert.ok(lines.some((line) => line.startsWith('damping,')));

  // 只有一种标签时另一种为空，但仍能用
  const onlyMovj = new TrajectoryCharts();
  onlyMovj.recordSample(0, { x: 0, y: 0, z: 0 }, 0);
  onlyMovj.recordSample(0.1, { x: 0.1, y: 0, z: 0 }, 1);
  const single = onlyMovj.buildComparison();
  assert.equal(single.straightness.MOVL, null);
  assert.equal(single.damping.samples.length, 0);

  // 空对象返回 null，不抛错
  assert.equal(new TrajectoryCharts().buildComparison(), null);
}

console.log('PASS: trajectory-analysis.test.mjs: 直线度偏差（理想直线/绕行/显式弦线/空输入）、各轴角速度（恒定/断档/单帧）、奇异区 DLS 阻尼（区间合并/统计）、对比汇总表与 CSV 导出。');
