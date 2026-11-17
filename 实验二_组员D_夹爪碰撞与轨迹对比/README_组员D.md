# 组员 D（王静）交付说明 —— 夹爪碰撞升级与 MOVJ/MOVL 轨迹对比

本目录是实验二组员 D 的完整交付，可直接复制进仓库的一个顶层目录（建议 `实验二_组员D_夹爪碰撞与轨迹对比/`）。

```
实验二_组员D_夹爪碰撞与轨迹对比/
├─ Animation/                         完整工程（基于组员 C 的包：组长底座 + 组员 A + 组员 C）
│  ├─ js/pendant-motion.js            ★ D 改动：夹爪碰撞检测
│  ├─ js/analysis/TrajectoryCharts.js ★ D 改动：直线度/角速度/DLS 阻尼三组曲线
│  ├─ js/analysis/analysis-panel.js   ★ D 改动：对比面板（动态创建，不改 index.html/css）
│  ├─ js/tests/gripper-collision.test.mjs   ★ D 新增
│  └─ js/tests/trajectory-analysis.test.mjs ★ D 新增
├─ 实验二报告第5章_夹爪碰撞与轨迹对比.md
├─ 第五章_图/                          图 5-1~5-5（PNG）+ movj/movl/damping/gripper（CSV）+ data.json
└─ tools/                              数据与图表复现脚本
```

## 一、组长合并时只需要 5 个文件

| 文件 | 说明 |
| --- | --- |
| `js/pendant-motion.js` | 新增 `checkGripperCollision()` 与 `resolveGripperScene()`，并入 `isColliding()` 第 5 段；`createMotion` 第三个参数可选，两参数调用行为不变 |
| `js/analysis/TrajectoryCharts.js` | 旧接口 `recordSample / renderCharts / exportToCSV` 不变，新增 4 个纯函数与 6 个方法 |
| `js/analysis/analysis-panel.js` | 只在「轨迹分析」页签下动态插入对比面板，未改 `index.html` / `css/style.css` |
| `js/tests/gripper-collision.test.mjs` | 10 组断言 |
| `js/tests/trajectory-analysis.test.mjs` | 8 组断言 |

## 二、验收步骤

1. VS Code 打开 `Animation/`，Live Server 或 `python -m http.server 5500`，访问 `index.html`；
2. 勾选「关节使能」；
3. 打开「轨迹分析」页签，滚动到下方「MOVJ vs MOVL 对比」面板；
4. 点「开始记录」→ 执行一段 MOVJ → 再执行一段 MOVL（DSL 的 `MOVL Pn`）→ 停止，自动生成对比；
5. 「生成对比」看三张曲线与汇总表，「导出对比 CSV」得到 `MOVJ_MOVL对比.csv`；
6. 移动机械臂让夹爪靠近工作台，触发碰撞时状态栏会出现原因（`state.collisionReason`）。

## 三、测试

在本机受限环境下 `node --test` 会因创建子进程被拒，逐个执行等价：

```bash
node js/tests/gripper-collision.test.mjs
node js/tests/trajectory-analysis.test.mjs
node js/tests/calibration.test.mjs
node js/tests/collision.test.mjs
node js/tests/group-b.test.mjs
node js/tests/ik.test.mjs
node js/tests/jog-mode.test.mjs
node js/tests/program.test.mjs
```

结果：8/8 通过（含原有 6 个回归测试）。

## 四、复现第 5 章数据与图表

```bash
node tools/chapter5-data.mjs     # 生成 第五章_图/ 下的 data.json 与 4 个 CSV
python tools/render_figs.py      # 生成 图5-1 ~ 图5-5
```

数据由 `Animation/js` 里的真实模块（`ik-solver.js`、`movl-planner.js`、`TrajectoryCharts.js`、`pendant-motion.js`）离线复算，运动链为 `js/tests/ik.test.mjs` 中的五轴测试链。

## 五、已知限制

1. 未录制演示视频/页面截图（制作环境无法启动浏览器）；正文已注明由操作者补录。
2. 本包基于组员 C 的分支（组长底座 + A + C），**未合入组员 B 的四点法/几何夹爪**；因此 `workstation.gripper` 不存在时，夹爪检测自动退回 `link_flange` 子树。合入 B 后自动改用 `gripper` + 已挂载工件。
3. 第 5.6 节记录了一个真实缺陷：带姿态约束时雅可比为 6×5，`J·Jᵀ` 行列式结构性为 0，导致 `singular` 恒真、DLS 阻尼恒为上限，建议组长评估改为 `√det(JᵀJ)`。
