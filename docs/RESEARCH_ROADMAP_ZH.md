# 从定位基础到 Gazebo SLAM 实验

本项目研究二维激光定位在场景变化下的表现，以及定位误差如何影响新增表面检测和构件安装偏差估计。现有 Python 实验是第一阶段基础。ROS 2、RViz、Gazebo 和定量对照实验是下面逐步实施的路线，完成情况以保存的日志与实验结果为准。

## 研究问题

1. ICP 相比带偏差里程计能够降低多少轨迹误差？低重叠和重复几何何时导致失败？
2. 新结构进入地图时，扫描匹配的误差与拒绝率如何变化？
3. 连续观测门限如何影响变化检测的精确率、召回率与检测延迟？
4. 定位误差与构件可见面数量如何影响安装位置和角度估计？

## 阶段一　理解并复现定位

按照根目录 README 配置 Python 环境。运行现有实验与测试，保留每次输出。

```bash
python -m construction_slam.demo --seed 7 --output artifacts/seed7
python -m unittest discover -s tests -v
```

阅读 core.py 的 SE(2) 变换、运动合成与 ICP。推导里程计预测如何给 ICP 提供初值，检查扫描点如何变换到地图坐标系。手算一个旋转90度的例子并与函数输出比较。

验收材料包括轨迹图、误差定义、运行命令、提交版本和失败案例。解释地图使用激光测量累积生成的过程，以及真值只用于仿真和评估的边界。

## 阶段二　建立可重复的对照实验

先运行相同配置下的多个随机种子。

```bash
for seed in 7 17 27 37 47; do
  python -m construction_slam.demo --seed "$seed" --output "artifacts/seed$seed"
done
```

比较里程计与现有扫描匹配。随后逐项修改噪声、初值误差、ICP 距离门限与裁剪比例。每次只改变一个因素，记录修改对应的 Git 提交。场景拓展加入走廊、遮挡与低重叠轨迹。

报告各次结果、均值、标准差和最差值。当前 shared-origin RMSE 不进行轨迹后对齐；与其他实现比较时必须统一时间戳、初始坐标系和采样区间。失败帧和回退里程计帧也进入评分。

## 阶段三　ROS 2 消息与 RViz

目标环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。先完成 ROS 2 官方安装与基础教程，再检查传感器消息。

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /clock --once
```

理解 LaserScan 的角度、量程、frame_id 和时间戳，理解 Odometry 的父子坐标系。检查发布者与订阅者 QoS 是否兼容。

现有适配器的运行方式和约束见 [ROS 2 接入说明](ROS2.md)。它要求激光与机器人基座重合，仅发布轨迹。运行它之前必须满足传感器外参和时间匹配要求。

RViz 中以 demo_map 为 Fixed Frame，添加 /demo/path 的 Path 显示。进一步显示 LaserScan 需要正确的 TF 链。保存 RViz 配置和截图，记录每一个坐标系的含义。

本阶段验收是能够解释一帧扫描从接收、匹配到轨迹发布的过程，并展示 topic、时间戳和 TF 检查记录。

## 阶段四　Gazebo 仿真与数据记录

从 Nav2 官方 Jazzy 仿真示例开始，按其安装页配置机器人和依赖。先启动机器人、激光、里程计与仿真时钟，检查静止和运动时的数据。

先用标准机器人完成成熟算法基线。接入本仓库适配器前，要实现实际激光外参变换和所需 TF；标准机器人的激光通常与基座存在偏移。

所有相关 ROS 节点使用仿真时间。记录一次固定路线的传感器数据。

```bash
mkdir -p bags
ros2 bag record -o bags/baseline /scan /odom /tf /tf_static /clock
```

用 Ctrl+C 正常结束记录，再执行 ros2 bag info bags/baseline。回放时停掉在线仿真与其他同名数据发布者，确保只有一个时钟来源。记录固定路线、机器人模型、软件版本和传感器配置。

验收材料包括 Gazebo 与 RViz 同时运行的录像、bag 信息、TF 树与时间同步检查。轨迹评分还需要额外记录仿真真值，并与估计轨迹对齐到相同初始参考系。

## 阶段五　SLAM Toolbox 建图与 AMCL 定位

按 Nav2 官方 SLAM 教程启动 SLAM Toolbox 和导航仿真。先缓慢绕场景一圈并返回起点，观察地图闭合与轨迹修正。

地图发布正常后，在仓库目录保存栅格地图。

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/baseline --ros-args -p use_sim_time:=true
```

保存的 YAML 和图像用于地图服务器与 AMCL。SLAM Toolbox 的位姿图序列化是另一种输出，继续建图需要按其文档单独保存。

结束建图运行后，按照 Nav2 定位流程重新启动仿真、地图服务器和 AMCL，加载 baseline.yaml。在 RViz 用 2D Pose Estimate 设置初始位姿，观察激光与地图重合情况，再给出导航目标。

验收分为建图、地图重新加载、初始定位、运动中持续定位和目标到达。每项分别记录结果。检查 map → odom → base_link → laser 的 TF 链，确认每段变换只有正确的发布者。

## 阶段六　变化场景研究

固定路线和传感器设置，分别记录静态场景与增加结构后的场景。第一轮只验证新增结构，后续再考虑移动物体和结构移除。

| 实验因素 | 对照方式 | 记录结果 |
|---|---|---|
| 定位方法 | 里程计、当前 ICP、SLAM Toolbox | 轨迹误差、失败率、运行时间 |
| 几何变化 | 静态、增加结构 | 变化前后定位误差 |
| 连续观测门限 | 1、3、5 帧 | 精确率、召回率、延迟 |
| 可观测性 | 一个面、两个面、遮挡 | 安装偏差误差、拒绝率 |

使用仿真真值构建变化标签，说明评估区域、可见性和点/栅格匹配容差。未观测区域不应直接标作新增结构的漏检。延迟从新增结构首次可被传感器观测时起算。

构件位置误差以米报告，角度误差处理周期性与矩形对称性。对 insufficient_evidence 单独计数，同时报告有效估计覆盖率。

## 实验记录模板

每次实验记录以下字段并链接实际产物。

- 日期、Git commit、Ubuntu/ROS/Gazebo 版本
- 场景、路线、种子、算法参数和传感器配置
- 运行命令、输入 bag、真值来源与坐标系约定
- 轨迹误差、失败率、运行时间、变化检测结果
- 截图或录像、异常日志、观察与下一次改动

研究阶段中的假设与预期结果应在运行前记录，实测结果运行后填写。仓库当前公开数值仍来自 README 中说明的 Python 合成场景。

## 参考资料

- [ROS 2 Jazzy 教程](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Gazebo Harmonic 与 ROS 安装](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Nav2 Jazzy 环境安装](https://docs.nav2.org/jazzy/getting_started/build_and_install/local_installation/)
- [Nav2 仿真配置](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/)
- [Nav2 使用 SLAM](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
