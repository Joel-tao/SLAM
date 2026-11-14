# CV与申请表达

原始CV中的Bosch经历已支持Python、OpenCV/PyTorch、传感器标定、噪声分析和SciPy非线性最小二乘，适合优先呈现。把长段落压缩成3—4条，避免没有指标支持的“significantly improving”。

建议顺序：Education → Technical Skills → Selected Projects → Bosch Experience → Selected Robotics Experience。保留视觉小车和工业机器人集成，压缩不相关活动及办公软件技能。核实学校正式学位名称、日期、个人贡献及C++熟练度。

附件中的旧建议提及3DGS场景图，但原始CV没有相应证据；请按真实做过的范围自行补充，不能把旧建议当作结果证明。

目前可用的项目表述（应先运行和理解）：

**2D LiDAR Mapping and Construction-Change Detection — Learning Project (AI-assisted)**
- Reproduced and studied a Python simulation combining noisy odometry, trimmed ICP scan-to-map registration, and incremental point-cloud mapping.
- Evaluated trajectory error against simulation ground truth and explored persistent geometric change detection against a frozen baseline.

亲自完成参数实验、修改和ROS2联调后，按实际完成内容更新动词与结果；当前不要写“Developed and deployed a complete ROS2/Gazebo SLAM system”。《SLAM十四讲》列为理论学习，不等同于VIO/LIO工程经验。

可在联系邮件中询问兼职安排：

> Would you consider a part-time Research Assistant arrangement during the academic term? My availability is up to 16 hours per week during term, and I would be interested in full-time work during the official university vacation, subject to confirmation by NUS HR.

这段表达你的可用时间，不判断工作资格。具体假期日期和可聘用安排由学校/HR确认；不要直接把整个12月到1月都写成假期。本次没有修改原CV，也没有发送或创建邮件。

## 本次新增的项目能力

加入了已知尺寸二维矩形构件的设计对比：鲁棒最小二乘拟合位姿，计算位置/角度偏差，并导出供规划器读取的JSON状态。设计与地图预先对齐，尚未连接真实规划器。

理解并复现后，可添加：

- Evaluated known-size 2D element pose fitting against a pre-aligned design, with placement-deviation checks and structured construction-state reports.

不要写成已完成CAD/BIM配准、6DoF构件跟踪或自主施工闭环。详见 [设计检查说明](INSPECTION_ZH.md)。
