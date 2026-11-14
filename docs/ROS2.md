# ROS2 integration (not runtime tested)

The host has Ubuntu 20.04 / ROS Noetic. It cannot validate this ROS2 adapter. Use a separate Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic environment for the next stage. Official compatibility: https://gazebosim.org/docs/harmonic/ros_installation/

The adapter expects:

- `/scan`: `sensor_msgs/msg/LaserScan`, meters and radians, planar sensor.
- `/odom`: `nav_msgs/msg/Odometry`, continuous odometry poses, same clock as scans.
- Lidar and robot base coincident in position and orientation. Nonzero extrinsics MUST be implemented before using another robot.
- Slow motion; no scan deskew. Nearest buffered odometry must be within 100 ms. Scans without matching earlier-arriving odometry are skipped; no waiting or interpolation.

LaserScan reference: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg

After installing ROS2 and NumPy/SciPy into a compatible Python environment, from repository root:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m ros2.scan_mapper --ros-args -p use_sim_time:=true
```

Provide `/clock` from simulation. In RViz choose `demo_map` as fixed frame, add Path `/demo/path`. The adapter publishes a path only; it does not publish TF, a map, or change markers. Do not connect it to Nav2 as a localization provider.

Next integration milestones (not completed):

1. Create a differential-drive Gazebo robot and world with lidar and odometry plugins.
2. Bridge clock, laser, odometry, commands; inspect `ros2 topic hz /scan` and message frames.
3. Add lidar extrinsic TF handling and timestamp interpolation.
4. Run this adapter and compare against Gazebo truth in an aligned initial frame.
5. Add `slam_toolbox` as a mature baseline; record rosbag2, test map export and relocalization.
6. Insert a structural element, assess change detection false positives and latency.

Never label the current demo “Gazebo validated” or “ROS2 deployed” until those runs succeed.
