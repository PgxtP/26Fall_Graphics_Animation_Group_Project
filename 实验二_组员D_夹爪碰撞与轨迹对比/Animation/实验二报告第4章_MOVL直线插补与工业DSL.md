# 第 4 章 MOVL 直线插补与工业 DSL 执行机

## 4.1 任务目标

本章在实验一「MOVJ 示教程序」的基础上增加两项能力：一是笛卡尔直线插补 MOVL，使 TCP 沿起终点连线以给定速度匀速直线运动、末端姿态连续平滑过渡；二是工业 DSL 指令集与执行状态机，使用户可以用 MOVJ、MOVL、GRIP、WAIT、LOOP、SET SPEED 六类指令编写抓取与搬运程序。

实验二底座为五轴机械臂，末端姿态只有两个自由度可调，因此 MOVL 采用位置线性插值加姿态四元数插值的方式生成中间位姿，每个中间位姿都由组长提供的逆解引擎求解。规划阶段逐点调用 `solveIK()` 做可达性预检，遇到不可达点或关节分支跳变时在运动开始之前给出错误，避免机械臂走到中途才失败。

本章不修改 `js/movj-program.js`、`js/pendant-motion.js`、`js/ik-solver.js`、`index.html` 和 `css/style.css`，原有 MOVJ 程序的编译与执行行为保持不变。

## 4.2 总体设计

DSL 程序的执行流程如下：

1. 操作者在上方记录若干示教位姿 P1、P2……，每个位姿包含五关节角度和 TCP 世界位姿。
2. 操作者在 DSL 文本框中编写程序，或点「插入 MOVL 示例」载入示例。
3. `parseDSLScript()` 将文本解析为指令数组，循环在解析期展开，语法错误带源文件行号返回。
4. 点「执行 DSL 程序」后，`createDSLRunner()` 创建的执行器进入 RUNNING 状态，并在运动层记录当前 `stopVersion`。
5. 执行器每帧由 `program.update()` 推进一次：上一条指令完成后才下发下一条。
6. MOVJ 指令把位姿的关节角直接交给 `moveJ()`；MOVL 指令先调用 `planMOVLTrajectory()` 生成插补点，再把点列按关节空间逐段下发给 `moveJ()`。
7. GRIP、WAIT、SET SPEED 分别操作夹爪、计时和速度倍率。
8. 急停、使能丢失、手动停止或碰撞回滚都会中断程序，执行器回到 IDLE 或 E-STOP。

规划、解释、界面三者分层，分别位于 `js/movl-planner.js`、`js/dsl-interpreter.js` 和 `js/program-panel.js`。规划器不引用任何其他模块，连 three.js 都不依赖，因此可以在 Node 中直接测试。

运动层只提供 `moveJ(angles)` 一个运动原语，内部为单段平滑插值，没有笛卡尔运动接口。MOVL 的实现方式是把一条直线离散成足够密的关节空间点列，由执行器在每一小段到达后下发下一小段。关节空间中小段连续平滑，笛卡尔空间中点间距不超过 0.05 个资产单位，宏观轨迹即为直线。

## 4.3 MOVL 直线插补

### 4.3.1 位姿表示

起点与终点统一表示为 `{ position: {x, y, z}, quaternion: {x, y, z, w} }`，同时兼容三种输入写法：直接给出 position 与 quaternion；给出 three.js 矩阵（`poseFromMatrix()` 取第 13 至 15 个元素为平移）；只给 position 时姿态沿用起点姿态。

示教位姿记录的是关节角，而 MOVL 需要 TCP 世界位姿，因此界面侧提供 `poseOfAngles()` 适配器：临时把关节角写入关节对象，刷新 `matrixWorld` 后读取 TCP 位姿，最后在 `finally` 中恢复原关节角并再次刷新矩阵。这一写法与 `js/cartesian-scan.js` 的扫描采样一致，规划期间画面上的机械臂保持不动。

### 4.3.2 位置线性插值与姿态插值

设起点、终点位置为 \(\mathbf p_0\)、\(\mathbf p_1\)，姿态四元数为 \(q_0\)、\(q_1\)，采样点数为 \(n\)，参数 \(t_i = i/(n-1)\)，则第 \(i\) 个插补点的位置为：

\[
\mathbf p_i = (1 - t_i)\mathbf p_0 + t_i \mathbf p_1
\]

在直线段上线性参数与弧长严格成正比，因此位置线性插值本身就是等弧长参数化，不需要额外的弧长表。姿态采用球面线性插值：

\[
q_i = \frac{\sin((1-t_i)\theta)}{\sin\theta} q_0 + \frac{\sin(t_i\theta)}{\sin\theta} q_1,\qquad \cos\theta = q_0 \cdot q_1
\]

两四元数点积为负时先将 \(q_1\) 取反，保证姿态沿最短弧过渡。

采样点数由线段长度 \(L\) 决定：

\[
n = \mathrm{clamp}\left(\left\lceil \frac{L}{L_{\max}} \right\rceil + 1,\ n_{\min},\ n_{\max}\right)
\]

默认 \(L_{\max} = 0.05\)、\(n_{\min} = 8\)、\(n_{\max} = 160\)。采样密度与速度无关，速度只影响时长估计，不改变轨迹形状。

### 4.3.3 逐点逆解预检与热启动

对每个插补点调用 `solveIK(pos, quaternion, previousAngles, options)`，传入的 `pos`、`quaternion` 就是 4.3.2 生成的第 \(i\) 个插补点位姿，`options` 默认取 `{ method: "dls", posTol: 1e-3, rotTol: 2e-2, maxIterations: 60 }`。逆解同时约束位置与姿态：`converged` 只有在位置误差不超过 `posTol`、并且给定姿态时姿态误差不超过 `rotTol` 才为 true（判定在 `js/ik-solver.js` 的数值求解分支内）。

`rotTol = 2e-2 rad` 约 1.1°。五轴末端只有两个姿态自由度，而中间点姿态由两端四元数做全姿态 SLERP 得到，不一定落在五轴可达的姿态流形上；因此预检把姿态超差同样视为不可达，让程序在机械臂开始运动之前整体失败，而不是带着姿态偏差走完直线。

求解以上一个插补点的关节角作为初值，即热启动。热启动一方面减少迭代次数，另一方面使相邻解留在同一个逆解分支上，这是直线运动不出现「翻腕」的关键。逆解引擎内部在 `finally` 中恢复实时关节角，因此规划阶段不会改动场景。

### 4.3.4 不可达与关节突跳的拦截

规划器对每个插补点记录 `converged`、`posError`、`rotError`、`singular` 等返回值，并按下表判定：

| 现象 | 判定条件 | 处理 |
| --- | --- | --- |
| 不可达 | `converged` 为 false（含位置或姿态任一项超差），或 `posError` 大于 `posTol × 1.5` | 拦截，返回 `ok: false` 与失败点序号，保留失败点之前的采样 |
| 关节突跳 | 相邻插补点任一关节变化大于 45° | 拦截，判定为逆解切换到了其他分支 |
| 奇异位形 | `singular` 为 true | 只计数，不拦截 |

奇异位形只计数不拦截，原因是逆解引擎在迭代过程中阻尼权重低于阈值时就会置 `singular`，收敛点也可能带该标记。若见到标记即拒绝，直线会在完全可用的构型附近频繁失败。奇异信息记录在规划结果的 `singularCount` 中，用于说明规划质量。

规划结果包含 `ok`、`reason`、`samples`、`distance`、`duration`、`maxPosError`、`maxRotError`、`maxJointStepDeg`、`singularCount` 等字段，每个采样点包含位置、四元数、关节角和解算统计量。

## 4.4 工业 DSL 指令集与解析器

### 4.4.1 指令集

| 指令 | 语法 | 含义 |
| --- | --- | --- |
| MOVJ | `MOVJ P3` | 关节空间点到点运动 |
| MOVL | `MOVL P2` | 笛卡尔直线插补，起点为运行时 TCP 当前位姿 |
| GRIP | `GRIP 1` / `GRIP 0` | 夹紧 / 松开，也接受 ON、OFF、CLOSE、OPEN |
| WAIT | `WAIT 0.5` | 等待若干秒，上限 60 秒 |
| SET SPEED | `SET SPEED 40` | 设置速度倍率 40%，运行期限制在 10 至 100 |
| LOOP … END | `LOOP 3` … `END` | 循环体，可嵌套 |

### 4.4.2 循环展开

`LOOP` 采用解析期展开：`LOOP 3` 内的三条指令展开为九条，每条都保留原源文件行号。采用展开而不是运行期计数，是因为执行器只需保存「当前执行到第几条」这一个游标，暂停、继续、急停都不必维护循环栈。展开后的总指令数受 500 条上限保护，超出时报错而不是静默截断。

### 4.4.3 错误定位

解析失败返回 `{ ok: false, errors: [{ line, message }] }`，`line` 为源文件行号，`0` 表示与具体行无关的整体性问题。已覆盖的错误包括：引用未记录的位姿、`END` 没有对应的 `LOOP`、`LOOP` 缺少配对的 `END`、无法识别的指令、`WAIT` 秒数越界、速度倍率越界、循环次数越界、程序为空、指令数超出上限。

## 4.5 执行状态机

### 4.5.1 状态定义

| 状态 | 含义 | `state.running` |
| --- | --- | --- |
| IDLE | 未启动或已停止 | false |
| RUNNING | 正在执行 | true |
| PAUSED | 暂停 | true |
| E-STOP | 急停或使能丢失 | false |
| DONE | 正常执行完毕 | false |

`state.running` 在暂停时仍为 true，因为 `js/analysis/analysis-panel.js` 依据该字段判断程序是否结束并停止采样，若暂停时置为 false，暂停会被误判为程序结束。

### 4.5.2 指令推进

执行器的 `update()` 每帧被调用一次，语义是「上一条指令已完成，则下发下一条」：

1. 状态不是 RUNNING 时直接返回。
2. 检查 `motion.allowed()`，失能或急停立即转入 E-STOP。
3. 检查 `motion.state.stopVersion`，与启动时记录的值不同说明被外部中断（碰撞保护回滚也会调用 `motion.stop()` 而改变该值），转入 IDLE。
4. 运动层 `busy()` 为真时等待，不重复下发。
5. 取当前指令执行：MOVJ 下发关节角；MOVL 生成点列后逐段下发；GRIP 操作夹爪；WAIT 记录截止时刻；SET SPEED 修改速度倍率。
6. 指令全部完成时转入 DONE，并在状态栏显示「程序执行完成，共 N 条指令」。

MOVL 的名义线速度为「1 个资产单位每秒乘以速度倍率」，用于估计时长和采样间隔；实际执行时长由运动层 30 度每秒的关节限速决定。

### 4.5.3 与既有程序执行器的兼容

新执行器的对外形状与原 `createProgramRunner()` 一致：

```js
api.program.runner.state.running;      // 是否在运行
api.program.runner.state.instructions; // 指令数组，位姿指令带 angles
api.program.runner.state.index;        // 当前指令序号
api.program.runner.update();           // 每帧推进
api.program.runner.stop();             // 停止
```

`api.program` 另外增加了 `dslCode` 与 `parseDSL()`，原有 `runner`、`poses`、`code` 三项保持不变。

## 4.6 界面接入

DSL 面板由 `js/program-panel.js` 在 `bindProgram()` 中动态创建并插入到示教编程列的后方，没有修改 `index.html`。面板包含文本框、插入示例、检查语法、执行、暂停、继续五个按钮，一行状态显示和一份展开后的指令清单。

状态行显示当前状态、指令序号、源文件行号和最近一次 MOVL 的插补点数，例如「状态机：RUNNING · 第 4/9 条 · 源文件第 6 行 · MOVL 24 个插补点」。执行失败时，失败原因直接显示在状态行，例如「无法执行：第 2 行：引用了未记录的位姿 P1」，同时写入原有的 `#program-status`。

运行期间录制、撤销、清空、生成、扫描等按钮被禁用，两个程序文本框设为只读；暂停与继续按钮按状态互斥启用。执行按钮在程序运行期间禁用，其余时间可点击，由执行器校验使能状态并给出具体原因。

## 4.7 独立测试设计

`js/movl-planner.js` 与 `js/dsl-interpreter.js` 都不依赖浏览器 DOM，可直接在 Node 中加载。为不改变团队既有的测试目录约定，本次未新增 `js/tests/*.test.mjs` 文件，改用临时脚本验证，覆盖以下内容：

1. 四元数球面插值的端点、中点数值；
2. 矩阵到位姿的换算；
3. 直线规划的采样数、位置单调前进、直线度与时长；
4. 不可达点的拦截、失败点序号与失败前采样的保留；
5. 关节突跳的识别；
6. 速度为 0 等非法参数；
7. 六类指令的解析、行尾注释与行号定位；
8. 嵌套循环的展开条数与重复次数；
9. 六类语法错误的报错文案；
10. 完整程序的执行、夹爪顺序与速度倍率还原；
11. 暂停、继续、急停、手动停止四种状态转移；
12. 未使能时拒绝启动、语法错误时抛出首条错误。

临时脚本以假机械臂驱动执行器：`moveJ()` 仅记录目标角并在下一个 tick 视为到位，夹爪动作被记录成调用序列，`solveIK` 用解析式假解替代，以便单独检查状态机逻辑。

## 4.8 测试结果

| 测试项目 | 结果 |
| --- | --- |
| 规划器与执行器临时自测 | 15/15 通过 |
| `program.test.mjs`、`jog-mode.test.mjs`、`ik.test.mjs`、`calibration.test.mjs`、`collision.test.mjs`、`group-b.test.mjs` 回归 | 19/19 通过，0 失败 |
| `js/program-panel.js` 模块语法检查 | 通过 |
| 面板引用的 DOM 标识符与 `index.html`、动态面板定义比对 | 15 个引用全部有定义 |
| 页面端按钮与状态行验收 | 按 4.6 节步骤由操作者逐项执行 |

回归测试表明，组长的逆解基准、组员 A 的点动与 HUD、组员 B 的坐标验证以及原有 MOVJ 程序执行均未受影响。

## 4.9 联调说明

组长合并时只需保留 `bindProgram(api, motion)` 的现有调用方式，`js/robot-model.js` 继续按原逻辑把返回对象挂到 `window.robotSimulation.program`，无需修改 `main.js`。

```js
const program = bindProgram(loaded, pendant);
```

夹爪指令在**执行时**才读取 `api.workstation`，原因是 `js/robot-model.js` 中 `bindProgram()` 的调用早于 `new Workstation(...)`，若在初始化时缓存工作站对象，拿到的始终是 undefined。GRIP 直接调用 `js/Workstation.js` 已提供的 `attachWorkpiece()` 与 `detachWorkpiece()`，并在动作后调用 `calibration.invalidateUserFrame()`，与页面上手工挂载工件的路径一致。

组员 D 的轨迹对比可以直接读取执行器状态：

```js
const { status, index, instructions, lastMOVL } = window.robotSimulation.program.runner.state;
// lastMOVL 为最近一次 MOVL 的规划结果：sampleCount、maxPosError、maxJointStepDeg、singularCount
```
