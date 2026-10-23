# 26秋_图形与动画_小组作业与实验报告

> **《图形与动画》课程实验一：机械臂仿真（一）——模型导入、关节运动控制与示教器界面**  
> **技术栈**：Vite + 原生 JavaScript (ES Module) + **Three.js r186** + WebGL2

---

## 📚 核心协作文档导航

1. **[`五人组接口与函数定义.md`](作业文档与分工/2026-10-23_小组作业多人分工方案_01/五人组接口与函数定义.md)**（**必读**：5 位组员各自负责的 `.js` 文件、函数签名与输入输出契约）
2. **[`小组作业多人合作分工指南.md`](作业文档与分工/2026-10-23_小组作业多人分工方案_01/小组作业多人合作分工指南.md)**（模块解耦架构图、协作流程与 4 阶段时间推进表）
3. **[`实验报告章节分工与撰写大纲`](实验报告/README_报告汇总与章节分工说明.md)**（第 1~5 章报告分工、采分点清单与提交说明）
4. **[`作业要求与行动清单.md`](作业文档与分工/2026-10-23_机械臂作业要求解读_01/作业要求与行动清单.md)**（实验一 PPT 讲义 33 页完整要求提炼、a~d 四项必做任务与验收标准）

---

## 👥 五人组模块分工与代码文件认领表

每个人**只修改自己负责目录下的文件**，不要直接修改他人的模块文件，写好后由 **1 号位** 在 [`src/main.js`](src/main.js) 中统一 `import` 总装：

| 组员编号 | 角色定位 | 负责源码文件（已建好函数骨架） | 负责核心功能 | 负责实验报告章节 |
| :--- | :--- | :--- | :--- | :--- |
| **👤 1 号位** | 主场景总装 + 3D 模型与正向运动学底座 | [`src/main.js`](src/main.js)<br>[`src/robot/RobotModel.js`](src/robot/RobotModel.js)<br>[`src/robot/Kinematics.js`](src/robot/Kinematics.js) | 1. 解析 GLB 机械臂关节树并驱动 J1~J6 旋转与限位保护 (`setJointAngles`)<br>2. 提取末端世界位姿、RPY 欧拉角与四元数 (`getTCPPose`)<br>3. 纯数学正向运动学矩阵推导与 $<10^{-6}$ 精度对拍验证 (`forwardKinematics` / `verifyFK`) | **第 1 章**：模型解析与 FK 正向运动学数学推导及三档误差验证表 |
| **👤 2 号位** | 示教器交互面板与安全状态机 | [`src/ui/TeachingBox.js`](src/ui/TeachingBox.js)<br>[`src/ui/teachingBox.css`](src/ui/teachingBox.css) | 1. 6 关节滑块与数值输入框（界面角度 $^\circ$ 转内部弧度 $\text{rad}$ 回调 `onJointChange`）<br>2. 屏幕 HUD 实时刷新末端坐标与姿态 (`updateTCPDisplay`)<br>3. 使能/失能、急停按钮状态机与 $10\% \sim 100\%$ 速度倍率条 (`onStateChange` / `getSpeedRatio`)<br>4. 自动播放时反向同步滑块 (`syncSliders`) | **第 2 章**：虚拟示教器交互架构、角度/弧度换算与急停安全控制流 |
| **👤 3 号位** | 坐标系三点法标定与拾取工位场景 | [`src/robot/Calibration.js`](src/robot/Calibration.js)<br>[`src/scene/Workstation.js`](src/scene/Workstation.js) | 1. 用户工件坐标系**三点法标定算法**（施密特正交化解算 $4\times 4$ 矩阵 `calibrateUserFrameThreePoints`）<br>2. 工具坐标系 TCP 偏置矩阵 (`createTCPOffset`)<br>3. 创建工业工作台与待抓取物块 (`initWorkstation`)<br>4. 切换物块父节点实现抓取吸附与释放 (`attachWorkpiece` / `detachWorkpiece`) | **第 3 章**：工具/用户坐标系三点法标定数学推导、验证数据与抓取工位设计 |
| **👤 4 号位** | 示教编程与 MOVJ 连续轨迹插值 | [`src/motion/PoseManager.js`](src/motion/PoseManager.js)<br>[`src/motion/MOVJPlanner.js`](src/motion/MOVJPlanner.js) | 1. 记录、删除、导出至少 4 个示教位姿点 (`savePose` / `exportProgramJSON`)<br>2. 加载位姿序列并启动连续播放 (`loadProgram` / `play`)<br>3. 基于帧间隔 $\Delta t$ 的关节空间平滑插值 (`update(deltaTime)`)<br>4. 播放暂停、恢复与急停中断 (`pause` / `resume` / `stop`) | **第 4 章**：MOVJ 关节空间平滑插值原理与连续执行状态机 |
| **👤 5 号位** | 可达区域点云、轨迹速度曲线与报告汇总 | [`src/analysis/WorkspaceCloud.js`](src/analysis/WorkspaceCloud.js)<br>[`src/analysis/TrajectoryCharts.js`](src/analysis/TrajectoryCharts.js)<br>[`实验报告/`](实验报告/README_报告汇总与章节分工说明.md) | 1. 关节限位内蒙特卡洛随机采样生成 `THREE.Points` 可达点云 (`generatePointCloud`)<br>2. 量化分析最大/最小可达半径与不可达空洞 (`analyzeWorkspaceGeometry`)<br>3. 高频采集末端数据并绘制 3D 轨迹图、位置曲线 $p(t)$ 与速度曲线 $v(t)$ (`recordSample` / `renderCharts`) | **第 5 章**：轨迹平滑性分析 + 可达区域空洞分析 + 汇总全组报告与演示视频 |

---

## ⚠️ 全组必须遵守的 4 条接口铁律

1. **空间坐标系**：统一使用 Three.js 右手坐标系（$X$ 轴向右红、$Y$ 轴向上绿、$Z$ 轴向外蓝）。
2. **物理单位约定**：**代码内部函数传参和计算一律使用【米 ($\text{m}$)】与【弧度 ($\text{rad}$)】**；仅在示教器 UI 界面显示时转换为毫米 ($\text{mm}$) 与角度 ($^\circ$)。
3. **关节数据格式**：统一使用长度为 6 的数组 `[j1, j2, j3, j4, j5, j6]`（单位：弧度）。
4. **代码隔离原则**：每人仅编辑自己负责的 `.js` / `.css` 文件（函数签名已在对应文件中预置好 `TODO` 注释），严禁改变对外导出的类名、方法名与返回值结构。

---

## 📂 仓库目录结构

```text
26秋_图形与动画_小组作业与实验报告/
├── README.md                          # 项目总览、五人分工表与运行说明（本文件）
├── index.html                         # 页面入口与 Canvas 容器
├── package.json                       # Vite 与 Three.js (r186) 依赖配置
├── public/
│   └── models/
│       └── robot3/                    # 6 自由度机械臂归一化资产包 (GLB + joint_map.json + URDF)
├── src/
│   ├── main.js                        # [1号位] 主场景初始化、灯光、相机与模块总装入口
│   ├── style.css                      # 全局基础样式
│   ├── robot/
│   │   ├── RobotModel.js              # [1号位] 机械臂骨骼层级驱动与 TCP 位姿提取
│   │   ├── Kinematics.js              # [1号位] 正向运动学 (FK) 纯数学矩阵推导与精度验证
│   │   └── Calibration.js             # [3号位] 工具坐标系偏置与用户坐标系三点法标定
│   ├── ui/
│   │   ├── TeachingBox.js             # [2号位] 虚拟示教器面板交互、角度/弧度换算与急停状态机
│   │   └── teachingBox.css            # [2号位] 示教器面板样式
│   ├── scene/
│   │   └── Workstation.js             # [3号位] 工业工作台场景与物块抓取/释放挂载
│   ├── motion/
│   │   ├── PoseManager.js             # [4号位] 示教位姿点记录与 JSON 导入导出
│   │   └── MOVJPlanner.js             # [4号位] MOVJ 关节空间平滑插值与连续播放控制器
│   └── analysis/
│       ├── WorkspaceCloud.js          # [5号位] 蒙特卡洛随机采样与可达工作空间点云分析
│       └── TrajectoryCharts.js        # [5号位] 末端轨迹采样记录、位置/速度曲线绘制与 CSV 导出
├── 实验报告/
│   └── README_报告汇总与章节分工说明.md  # 实验报告第 1~5 章撰写大纲与提交指南
└── 作业文档与分工/
    ├── 2026-10-23_小组作业多人分工方案_01/
    │   ├── 五人组接口与函数定义.md      # 详细接口契约、函数 I/O 签名与总装示例
    │   └── 小组作业多人合作分工指南.md  # 协作架构图与阶段推进计划
    └── 2026-10-23_机械臂作业要求解读_01/
        ├── 作业要求与行动清单.md        # 课程实验一 PPT 要求解读与验收标准
        ├── 课件逐页文字.json            # 实验一 33 页课件完整文字提取
        └── 原课件配图/                  # 课件参考示意图
```

---

## 🚀 本地快速启动指南

### 1. 克隆仓库并安装依赖
需提前安装 [Node.js](https://nodejs.org/)（建议 v18+）：

```powershell
git clone https://github.com/PgxtP/26Fall_Graphics_Animation_Group_Project.git
cd 26Fall_Graphics_Animation_Group_Project
npm install
```

### 2. 启动本地开发服务器
```powershell
npm run dev
```
在浏览器打开终端输出的本地地址（如 `http://127.0.0.1:5173/`），页面显示“机械臂模型加载成功 · WebGL2 可用”即表示环境正常。
- **重要提示**：由于使用 ES Module 与 GLB 模型异步加载，**必须通过 `npm run dev` 启动 HTTP 服务访问，严禁直接双击打开 `index.html`**（`file://` 协议会触发浏览器 CORS 跨域拦截）。

### 3. 构建与语法检查
提交代码前可执行以下命令检查模块导入与语法是否正确：
```powershell
npm run build
```
