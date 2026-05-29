# NTU 2024 ICRA 论文复现修改说明

本文档说明本次为了复现论文
`Planning Optimal Trajectories for Mobile Manipulators under End-effector Trajectory Continuity Constraint`
而在当前 `rapid-transport` 项目中新增的内容、实现思路、运行方式、输出文件和当前限制。

## 1. 修改目标

用户需求是：以当前项目环境为基础，使用 `UR3 + 移动底盘` 模型创建仿真场景，并复现论文中的方法。

论文的核心问题是：给定一个连续、带时间参数的末端轨迹，例如移动打印或喷涂任务，移动操作臂需要让末端沿该轨迹连续运动。论文采用解耦式方法：

1. 先在移动底盘的 configuration spacetime 中规划底盘轨迹。
2. 底盘轨迹必须满足末端连续轨迹对应的可达区域、碰撞约束和底盘速度约束。
3. 使用后向动态规划求解最小控制代价的最优底盘轨迹。
4. 在得到底盘轨迹后，再计算机械臂轨迹。

当前项目原本主要是 `OpenRAVE + RRT + TOPPRA/接触约束` 的快速搬运框架，已经包含 `UR3 + Ranger-style mobile base` 模型。因此本次新增了一个独立复现目录，不侵入原有搬运 demo 的主流程。

## 2. 新增目录

新增目录：

```text
reproductions/ntu_2024_icra_mobocontp/
```

该目录用于专门存放论文复现场景、配置、脚本、输出和说明文档。

目录结构：

```text
reproductions/ntu_2024_icra_mobocontp/
├── README.md
├── MODIFICATION_REPORT.md
├── run_mobocontp_ur3.py
├── config/
│   ├── exp_verify.yaml
│   ├── large_printing_world.yaml
│   └── printing_line.yaml
├── models/
│   ├── exp_verify_ur3_ranger.env.xml
│   ├── large_printing_world.env.xml
│   ├── open_workspace.env.xml
│   └── ur3_ranger_printing.env.xml
└── outputs/
    ├── exp_verify_trajectory.csv
    ├── exp_verify_dense_base_ee.csv
    ├── exp_verify_top_view.svg
    ├── exp_verify_summary.json
    ├── large_printing_world_trajectory.csv
    ├── large_printing_world_dense_base_ee.csv
    ├── large_printing_world_top_view.svg
    ├── large_printing_world_summary.json
    ├── printing_line_trajectory.csv
    ├── printing_line_dense_base_ee.csv
    ├── printing_line_top_view.svg
    └── printing_line_summary.json
```

## 3. 新增文件说明

### 3.1 `models/ur3_ranger_printing.env.xml`

这是本次新增的 OpenRAVE 仿真场景。

主要内容：

- 加载项目已有的 `models/ur3/ur3.robot.xml`。
- 使用该模型中的 UR3 机械臂和 Ranger 风格移动底盘。
- 添加一个低矮打印平面 `printing_surface`。
- 添加一个静态障碍物 `base_obstacle`，用于让底盘规划考虑避障。
- 设置了初始 UR3 关节值，便于场景加载后有稳定姿态。

该文件只创建场景，不实现规划算法。

### 3.2 `models/exp_verify_ur3_ranger.env.xml`

这是新增的验证实验 OpenRAVE 场景。

主要内容：

- 复用同一个 `UR3 + Ranger-style mobile base` 模型。
- 添加更宽的打印平面 `exp_verify_printing_surface`。
- 添加两个可视化障碍物 `exp_verify_block` 和 `exp_verify_side_block`。
- 障碍物与 `config/exp_verify.yaml` 中的底盘碰撞过滤区域对应，用于验证 DP 规划过程中 admissible B-spacetime 会被障碍物收缩。

### 3.3 `models/large_printing_world.env.xml`

这是新增的大型可视化仿真场景。

主要内容：

- 去掉了原先容易误解为障碍物的大打印平面。
- 仅保留一个很薄、低于地面的 `workspace_floor` 用作场地参考。
- 添加多个更符合移动操作任务的环境障碍：
  - `column_left`
  - `column_mid`
  - `column_right`
  - `wall_back`
- 场景尺度扩展到约 2 米级，使底盘需要跨越更长距离跟随连续末端轨迹。

### 3.4 `config/printing_line.yaml`

这是默认复现实验配置。

主要配置项：

- `scene`：指定 OpenRAVE 场景文件。
- `robot`：使用 `ur3`。
- `manipulator`：使用 `denso_suction_cup2` 作为末端执行器。
- `task`：定义连续末端轨迹。
  - 当前默认任务是一条带时间参数的直线打印轨迹。
  - 持续时间为 `12.0s`。
  - 采样间隔为 `1.0s`。
- `base_grid`：定义底盘离散网格。
  - 底盘状态为 `[x, y, yaw]`。
- `controls`：定义底盘速度控制集合。
  - 对应论文中的 admissible controls `Ua`。
- `cost`：定义代价权重。
  - 当前使用平移速度平方和 yaw 速度平方的加权积分。
- `reachability`：定义简化几何可达区域。
- `collision`：定义底盘障碍物区域。
- `ik`：定义是否启用 OpenRAVE IK。
  - 默认 `off`，用于快速复现底盘 spacetime 规划。
  - 可以通过命令行切换为 `load` 或 `generate`。
- `output`：定义输出路径和文件名前缀。

### 3.5 `config/exp_verify.yaml`

这是新增的验证实验配置。

与默认 `printing_line.yaml` 不同，`exp_verify.yaml` 使用：

- U 形连续末端轨迹。
- `polyline` 路径类型。
- 两个底盘障碍物。
- 更长任务时间 `24.0s`，让底盘有足够时间绕过受限区域。

该配置用于验证：脚本不仅能生成一条简单直线示例，还能在更复杂的连续末端轨迹和障碍物过滤下完成 MoboConTP 风格后向动态规划。

### 3.6 `config/large_printing_world.yaml`

这是新增的大型复杂任务配置。

与 `exp_verify.yaml` 相比，它使用：

- 更大的底盘搜索区域。
- 更长的蛇形连续末端轨迹。
- 多个离散障碍物和墙式障碍。
- `37` 个时间采样点。
- 更明显的底盘长距离移动。

该配置更接近论文中移动打印/喷涂类任务的目标：末端持续沿给定轨迹运动，移动底盘在较大工作空间内缓慢跟随。

### 3.7 `run_mobocontp_ur3.py`

这是本次新增的主要复现脚本。

它实现了一个独立的 MoboConTP 风格规划流程：

1. 读取 YAML 配置。
2. 生成连续末端轨迹采样。
3. 构造底盘状态网格。
4. 构造底盘可行速度控制集合。
5. 对每个末端轨迹时间点，构造可行底盘状态集合。
6. 使用后向动态规划求解最小代价底盘轨迹。
7. 导出 CSV、JSON 和 SVG 结果。
8. 可选：调用 OpenRAVE IK 为每个底盘状态求 UR3 关节解。
9. 可选：在 OpenRAVE viewer 中播放结果。

为了支持验证实验，本次还扩展了 `build_task_samples`，现在支持两种末端路径：

- `line`：由 `start` 和 `end` 定义。
- `polyline`：由多个 3D waypoint 定义，并按路径长度均匀采样。

脚本中重要函数对应关系：

| 脚本函数 | 作用 | 对应论文概念 |
| --- | --- | --- |
| `build_task_samples` | 采样连续末端轨迹 | time-parametrized end-effector trajectory |
| `base_grid_from_config` | 构造底盘配置网格 | discretized base configuration space |
| `build_admissible_controls` | 构造可行速度控制 | admissible controls `Ua` |
| `build_admissible_spacetime` | 构造每个时间步的可行底盘状态 | admissible B-spacetime `Xa` |
| `mobocontp_backward_dp` | 后向动态规划求最优轨迹 | Algorithm 1: MoboConTP |
| `control_cost` | 计算控制代价 | minimum control effort cost |
| `IkChecker` | 可选 IK 验证和机械臂解算 | manipulator trajectory planning |

### 3.8 `README.md`

这是复现目录的快速使用说明。

包含：

- 文件说明。
- 运行命令。
- 输出文件说明。
- 与论文方法的简要对应。
- 当前实现注意事项。

### 3.9 `outputs/`

这是脚本运行后的输出目录。

当前已经生成了一组默认实验结果：

- `printing_line_trajectory.csv`
  - 离散时间步上的最优底盘轨迹和末端位置。
  - 如果启用 IK，还会包含 `q1` 到 `q6`。
- `printing_line_dense_base_ee.csv`
  - 对底盘轨迹和末端轨迹做更细时间步插值后的结果。
- `printing_line_top_view.svg`
  - 俯视图。
  - 蓝色为连续末端轨迹，橙色为最优底盘轨迹。
- `printing_line_summary.json`
  - 记录配置路径、场景路径、状态数量、控制数量、代价、每阶段可行节点数量和输出路径。

同时已经生成一组验证实验结果：

- `exp_verify_trajectory.csv`
- `exp_verify_dense_base_ee.csv`
- `exp_verify_top_view.svg`
- `exp_verify_summary.json`

还生成了一组大型复杂任务结果：

- `large_printing_world_trajectory.csv`
- `large_printing_world_dense_base_ee.csv`
- `large_printing_world_top_view.svg`
- `large_printing_world_summary.json`

## 4. 与论文方法的对应

论文中的移动底盘状态：

```text
qb = (x, y, phi)
```

在本复现中对应：

```text
base = [base_x, base_y, base_yaw]
```

这里的 `base_yaw` 不是直接使用 OpenRAVE 模型原始 yaw，而是定义为用户确认的车头方向：靠近 `zhijia` 支架 / UR3 机械臂的一侧。经 OpenRAVE link/AABB 验证，脚本用 `robot.SetTransform(yaw=0)` 放置整机时，支架侧相当于模型局部 `-X` 方向，即相对 OpenRAVE 模型 yaw 偏转 `180 deg`。因此配置中使用：

```yaml
base_model:
  front_yaw_at_identity_deg: 180.0
```

脚本内部会把规划 yaw 转换为 OpenRAVE 模型 yaw：

```text
openrave_model_yaw = base_yaw - 180 deg
```

这样在规划、CSV 和俯视图中，`base_yaw = 0` 就统一表示“车头/车辆局部 +X 指向世界 +X”。

为了保证工作时车头 `+X` 与机械臂工作方向一致，`reachability.max_heading_error_deg` 现在约束：

```text
angle(base_yaw, atan2(ee_y - base_y, ee_x - base_x)) <= 90 deg
```

也就是末端目标必须落在车辆前半空间；同时 `cost.heading_weight` 会继续惩罚该夹角，使可行时优先选择更正对 TCP 的底盘朝向。

论文中的 configuration spacetime event：

```text
x = (t, x, y, phi)
```

在本复现中对应每个时间步 `i` 下的一个底盘网格节点：

```text
(sample[i].t, node.pose[0], node.pose[1], node.pose[2])
```

论文中的 admissible B-space `B_a(t)`：

在本复现中由以下条件共同确定：

- 底盘网格范围。
- 几何可达区域 `reachability`。
- 底盘障碍物过滤 `collision`。
- 可选 OpenRAVE IK 验证。

论文中的 admissible controls `Ua`：

在本复现中由配置项：

```yaml
controls:
  dvx
  dvy
  domega_deg
  vmax
  omega_max_deg
```

生成。

论文中的后向动态规划：

在本复现中由 `mobocontp_backward_dp` 实现。它从最后一个时间阶段开始，逐层计算哪些节点能到达终点阶段，同时记录 cost-to-go 和最优 successor。

论文中的代价函数：

论文使用最小控制努力形式。当前实现为：

```text
dt * (vx^2 + vy^2 + yaw_weight * omega^2)
```

其中 `yaw_weight` 来自配置文件。对可视化 NTU 桌面场景，还叠加了轻量的几何偏好项：`heading_weight` 让车头 `+X` 尽量指向当前 TCP 目标，`radial_weight` 让底盘与 TCP 保持更稳定的工作半径。

## 5. 运行方式

在项目根目录执行：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml
```

默认运行不启用 IK，因此不需要 OpenRAVE IKFast 缓存。该模式用于快速验证论文中的底盘 spacetime 规划方法。

如果要尝试为 UR3 生成完整关节轨迹：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
  --ik generate
```

注意：`--ik generate` 会让 OpenRAVE 为当前 manipulator 生成 IKFast，可能需要数分钟。

如果已有 IKFast 缓存，可以使用：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
  --ik load
```

如果要在 OpenRAVE viewer 中播放：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
  --view
```

运行新增验证实验：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/exp_verify.yaml
```

运行新增大型复杂场景：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/large_printing_world.yaml
```

## 6. 已验证结果

已运行默认命令并通过。

关键运行结果：

```text
Loaded task samples: 13
Base grid states per stage: 2835
Admissible controls: 105
Optimal base trajectory cost: 0.020000
```

新增验证实验也已运行通过：

```text
Loaded task samples: 25
Base grid states per stage: 2970
Admissible controls: 105
Optimal base trajectory cost: 0.012500
```

验证实验中，后向阶段的可达节点数量在中间时段明显收缩，例如第 7 阶段只有 `5` 个节点能到达目标，说明障碍物和连续末端轨迹确实在约束 admissible B-spacetime。

大型复杂场景已运行通过：

```text
Loaded task samples: 37
Base grid states per stage: 6665
Admissible controls: 105
Optimal base trajectory cost: 0.092500
```

该场景中底盘从 `x=0.55` 移动到 `x=1.60`，末端沿长蛇形轨迹连续运动，更适合作为可视化仿真展示。

联合可视化仿真 `arm_motion_demo.yaml` 已重新调整并运行通过：

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/arm_motion_demo.yaml \
  --ik load
```

关键结果：

```text
Loaded task samples: 13
Base grid states per stage: 150
Admissible controls: 259
Loaded valid voxel cloud: 5585 / 14157 valid voxels
Solved IK for 13/13 trajectory samples.
Optimal base trajectory cost: 0.032500
```

该 demo 在 DP 中加入底盘起终约束，起点为 `(0.35, 0.00, 180deg)`，
终点为 `(0.85, -0.25, 180deg)`。按照论文中的几何可达区域思路，脚本会先
在机械臂基座周围离线/缓存生成 valid voxel cloud，再把每个任务时刻的末端
目标点反投影到底盘候选姿态的局部坐标系中，查询该点是否落在有效体素云内，
由此生成 admissible B-spacetime。当前缓存为
`outputs/arm_motion_valid_voxel_cloud.npz`，包含 `14157` 个体素，其中 `5585`
个有效体素。验证输出显示底盘位移约 `0.49m`，UR3 关节也同步变化，
最大相邻采样关节变化约 `0.51rad`。OpenRAVE viewer 回放阶段额外做了
底盘和关节插值，因此画面不是逐采样跳变。当前该 demo 使用
`models/open_workspace.env.xml`，场景中只保留 UR3+底盘和薄地面，不显示柱子、
墙体或打印平台等障碍物。

每个阶段的 admissible base states 和 backward reachable nodes 都能正常生成，说明默认任务在当前离散化下可行。

论文同类型 U 形打印轨迹也已添加为 `config/paper_u_shape.yaml`。论文 Fig. 7 报告的
硬件 demo 是 `0.9 x 0.675 x 0.05m`、5 层、总打印路径 `19.85m`、喷嘴速度
`10cm/s`、`dt=3s`、`Delta vx = Delta vy = 5cm/s`、`Delta omega = pi/30rad/s`。
由于 PDF 未给出逐点 CAD 坐标，本配置生成同尺寸的多层 U 形轮廓，路径长度为
`19.844m`，平均速度约 `10.02cm/s`。运行结果：

```text
Loaded task samples: 67
Base grid states per stage: 529
Admissible controls: 259
Built valid voxel cloud: 5587 / 15925 valid voxels
Solved IK for 67/67 trajectory samples.
Optimal base trajectory cost: 0.037500
```

输出文件包括：

- `outputs/paper_u_shape_trajectory.csv`
- `outputs/paper_u_shape_dense_base_ee.csv`
- `outputs/paper_u_shape_top_view.svg`
- `outputs/paper_u_shape_summary.json`

论文 Fig. 1 的 NTU 打印轨迹已添加为 `config/paper_ntu_shape.yaml`。论文报告
NTU 形状为 `3.0 x 0.75 x 0.15m`、10 层、总打印路径 `112.9m`、喷嘴速度
`10cm/s`。由于 PDF 未给出逐点 CAD 坐标，本配置生成同尺寸、按 `N/T/U`
分字母打印的连续粗笔画路径，并通过 `target_total_length: 112.9` 匹配总路径长度。
当前生成结果为 `300`
个几何 waypoint，路径长度 `112.900000m`，平均速度约 `0.10009m/s`。验证结果：

```text
Loaded task samples: 95
Base grid states per stage: 697
Admissible controls: 259
Loaded valid voxel cloud: 573 / 2527 valid voxels
Solved TCP z-axis numeric IK for 95/95 trajectory samples.
Optimal base trajectory cost: 2.490000
```

关于“吸盘末端 Z 轴向下”：已以 OpenRAVE manipulator TCP 为准进行验证。
`denso_suction_cup2` 的 `manip.GetTransform()` 第三列即 TCP 局部 Z 轴，最终轨迹
使用 5D 数值 IK 精修，让 TCP local `Z` 轴指向世界 `-Z`，同时保留绕吸盘轴自转
自由度，符合论文中 5D 打印任务“位置 + 向量姿态”的设定。独立回放 CSV 检查结果：

```text
samples 95
max_tcp_z_down_error_deg 0.08679914125896969
mean_tcp_z_down_error_deg 0.002154504838523634
max_position_error_m 0.0005042818787580745
mean_position_error_m 2.3263889825911746e-05
```

为了更直观地查看“在桌子/平面上只画一次 NTU”的场景，新增了：

- `models/ntu_table_single.env.xml`
- `config/paper_ntu_table_single.yaml`

这个配置不堆叠 10 层，只在 `z=0.56m` 的单层路径上打印一次，桌面平面位于
`z=0.535m`，因此 OpenRAVE 里能清楚看到末端轨迹落在桌面上方。当前版本将
目标轨迹改为与用户提供俯视图一致的 CAD 风格 NTU 外形轮廓闭合环路：三个字符
宽度比例基本一致，N/T 由直线段组成，U 的底部由平滑圆弧/椭圆弧组成，整条
外轮廓从起点连续走一圈后闭合回起点。当前单层路径保持论文的
`3.0 x 0.75m` 外接尺寸，包含 `431` 个采样 waypoint，长度为 `11.2953m`，
接近论文十层总长度 `112.9m` 的单层长度 `11.29m`；闭合距离为 `0`，零长度
断点数量为 `0`。SVG/PNG 俯视图中以黄色粗线绘制连续打印轨迹，并在深灰桌面
/禁入区上方显示。底盘加入了桌面足迹禁入区和
`table_side_band` 引导，使底盘绕桌子上下两侧运行，而不是堆在桌面区域内。
为修正 OpenRAVE 中车头朝向，配置加入
`base_model.front_yaw_at_identity_deg: 180.0`。这表示以靠近 `zhijia` 支架 /
UR3 机械臂的一侧作为车头；脚本用 `robot.SetTransform(yaw=0)` 放置整机时，该
支架侧对应模型局部 `-X`，因此需要 `180 deg` 补偿。`base_yaw` 统一定义为
车头正方向；脚本只在 IK 求解和动画播放时使用
`openrave_model_yaw = base_yaw - front_yaw_at_identity_deg` 来摆放整台机器人，
用来补偿原始 Ranger 模型的可视化朝向。CSV 同时输出 `openrave_model_yaw`，
方便对照 OpenRAVE 内部实际使用的整机朝向。这个处理不修改底盘、支架、机械臂
各自的局部 mesh 旋转，因此不会破坏原始装配关系；全局
`models/ur3/ur3.robot.xml` 未被修改。

碰撞约束按论文中的 `B_free(t)` / `B_a^i` 方式实现：构造每个离散时刻的
admissible base set 时，先将桌面/桌下区域作为静态障碍，并用随 `base_yaw`
旋转的 Ranger 矩形 footprint 做相交检测，剔除发生碰撞的底盘配置；随后才进行
整机 OpenRAVE 几何碰撞检测，检查车体、`zhijia` 支架、UR3 各 link、吸盘工具
与桌面和桌边的碰撞；通过后才进行几何可达性和 DP 连接。当前桌面单层 NTU
场景中，`base_yaw=0` 表示车头朝世界
`+X`。经 OpenRAVE link/AABB 检查，`base_pose` 更接近 UR3 `base_link`/肩部
参考点；当车头定义为支架侧时，Ranger 车体中心相对车头参考点位于局部
`[-0.20, 0.00]`，因此桌面单层场景使用 `center_offset=[-0.20, 0.00]`。当前
footprint 设为
`half_extents=[0.18, 0.16]`、桌面 clearance 为 `0.02m`，用于让可视化规划
保留论文式桌面禁入区，同时不过度保守地把 UR3 推出可达范围。几何可达外半径
设为 `max_radius=1.30m`，但 `radial_weight=12.0` 会将底盘优先拉向
`target_radius=0.55m`。

为了满足“工作时车头 `+X` 和机械臂工作方向一致”，配置中
`reachability.max_heading_error_deg=90.0` 作为硬约束。为了减少工作时车体大幅
旋转，当前配置将 `yaw_weight` 提高到 `20.0`，并将 `heading_weight` 降为
`1.5`，即优先少转，仍保持车头在 TCP 前半空间内。当前输出的实际轨迹最大夹角
为 `15.03 deg`，平均夹角为 `4.72 deg`；yaw 非零转向次数为 `13` 次，总转角
为 `240 deg`，最大单步转向为 `30 deg`。

验证结果：

```text
Loaded task samples: 47
Base grid states per stage: 20424
Admissible controls: 897
OpenRAVE whole-robot collision checks: 82796, rejected 20028
Solved TCP z-axis numeric IK for 29/47 trajectory samples.
Keeping previous arm posture for failed samples: [0, 6, 7, 8, 15, 16, 17, 27, 28, 29, 30, 31, 35, 36, 37, 38, 43, 44]
Optimal base trajectory cost: 254.336568
Vehicle +X/work heading error: max 15.03 deg, mean 4.72 deg
Whole-robot OpenRAVE collisions: env [], self []
```

单独回放 `paper_ntu_table_single_trajectory.csv` 的检查结果：

```text
samples 47
missing_q []
base_x_range -2.0 2.0
base_y_range -0.975 0.9
base_inside_table_forbidden []
ntu_art_waypoints 431
ntu_art_loop_length_m 11.2953
ntu_art_closed_distance_m 0.0
ntu_art_zero_segments 0
duration_dt_samples 115.0 / 2.5 / 47
vehicle_x_work_heading_error_max_deg 15.03
vehicle_x_work_heading_error_mean_deg 4.72
base_yaw_nonzero_steps 13
base_yaw_total_rotation_deg 240.0
base_yaw_max_step_deg 30.0
whole_robot_env_collision_samples []
whole_robot_self_collision_samples []
ee_x_range -1.5 1.5
ee_y_range -0.375 0.375
ee_z_unique [0.56]
failed_numeric_ik_samples [0, 6, 7, 8, 15, 16, 17, 27, 28, 29, 30, 31, 35, 36, 37, 38, 43, 44]
```

注意：该单层桌面场景当前定位为可视化复现。为保证动画完整，配置启用了
`numeric_axis_refine.allow_partial: true`，数值 IK 未收敛的采样点会沿用上一帧
机械臂姿态；若上一帧姿态在当前底盘位姿下会产生 OpenRAVE 碰撞，则自动切换为
当前位姿下无碰撞的 nominal 姿态。底盘、桌面避障和末端 NTU 轨迹仍完整输出。
若需要严格机械臂全程 IK，可继续将这些失败采样点改为离线 valid voxel cloud
过滤后的底盘状态。

也单独验证了 OpenRAVE 场景可加载：

```text
load True
robot ur3
bodies ['ur3', 'printing_surface', 'base_obstacle']
manips ['denso_ft_sensor', 'denso_suction_cup', 'denso_suction_cup2']
```

## 7. 当前实现与论文原文的差异

本次复现重点是搭建可运行场景和复现论文的主要算法结构，因此存在以下简化：

1. 论文未给出 Fig. 1 的原始 NTU CAD 点表；当前 `paper_ntu_shape.yaml` 是按论文公布的尺寸、层数、总长度和图中字形拓扑生成的复建轨迹。
2. 默认简单 demo 仍可使用几何距离、高度、朝向过滤；`paper_u_shape.yaml`、`paper_ntu_shape.yaml` 和 `arm_motion_demo.yaml` 已加入 valid voxel cloud，可离线/缓存预计算有效体素云。
3. NTU 最终机械臂轨迹使用 5D 数值 IK 做 TCP Z-down 精修，而不是固定完整 6D 姿态；这更符合论文中喷嘴/吸盘轴向下、自转自由的任务模型。
4. 当前实现没有接入原项目的 TOPPRA 接触约束，因为该论文关注的是连续末端轨迹下的底盘 spacetime 最优规划，不是吸盘搬运接触稳定性。
5. 当前动态障碍物未实现，只实现了静态底盘障碍物过滤。

## 8. 为什么放在独立目录

本次复现没有修改原有 `transport/`、`toppra_sc/`、`rrt.py` 或既有 demo 入口，原因是：

- 原项目已有快速搬运和接触稳定性管线，和该论文的移动打印问题不是同一个实验入口。
- 独立目录可以避免破坏已有 demo。
- 新脚本可以清楚展示论文算法结构，便于后续继续扩展。
- 场景、配置、输出集中管理，复现实验更容易重复。

## 9. 后续可扩展方向

建议后续扩展：

1. 将 `reachability` 替换为真正的 UR3 体素 IK 可达云。
2. 将默认任务从 1D line 扩展为论文中的 U-shape 或 NTU-shape 多层打印路径。
3. 启用 `--ik generate` 后，将每个时间步的 UR3 IK 结果保存为完整 9D 轨迹。
4. 对比 MoboConTP 和 Dijkstra/RRT baseline 的规划时间与代价。
5. 增加动态障碍物或已打印结构带来的时变碰撞区域。
6. 将输出轨迹进一步接入原项目的 OpenRAVE 轨迹播放或 ROS 控制接口。

## 10. 环境说明

本次复现脚本运行所需主要依赖：

- Python 3.8
- NumPy
- PyYAML
- OpenRAVE/openravepy，只有在启用 IK 或 viewer 时需要

当前项目 `requirements.txt` 已包含 `PyYAML`。开发过程中还临时安装了 `pypdf` 用于读取本地论文 PDF，该库不是复现脚本的运行依赖。
