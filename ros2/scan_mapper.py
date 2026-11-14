#!/usr/bin/env python3
"""Experimental ROS2 adapter. Run from repo root: python -m ros2.scan_mapper.
Assumes planar lidar coincides with base frame; timestamps matched within 100 ms.
"""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from construction_slam.core import Mapper, relative


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class ScanMapper(Node):
    def __init__(self):
        super().__init__('educational_scan_mapper')
        self.mapper = Mapper()
        self.buffer = deque(maxlen=200)
        self.previous = None
        self.last_stamp = -float('inf')
        self.path = Path()
        self.publisher = self.create_publisher(Path,'/demo/path',10)
        self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.lidar,qos_profile_sensor_data)

    def odometry(self,msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.buffer.append((seconds(msg.header.stamp),np.array([p.x,p.y,yaw])))

    def lidar(self,msg):
        stamp = seconds(msg.header.stamp)
        if not self.buffer or stamp <= self.last_stamp:
            return
        time, odom = min(self.buffer,key=lambda item: abs(item[0]-stamp))
        if abs(time-stamp) > .1:
            self.get_logger().warning('Skipping scan: no odometry within 100 ms')
            return
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges)&(ranges>msg.range_min)&(ranges<msg.range_max)
        scan = np.column_stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])])
        if len(scan)<30:
            return
        motion = np.zeros(3) if self.previous is None else relative(self.previous,odom)
        pose = self.mapper.update(scan,motion)
        self.previous, self.last_stamp = odom.copy(),stamp
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'demo_map'
        out.pose.position.x,out.pose.position.y = float(pose[0]),float(pose[1])
        out.pose.orientation.z = math.sin(pose[2]/2)
        out.pose.orientation.w = math.cos(pose[2]/2)
        self.path.header = out.header
        self.path.poses.append(out)
        self.path.poses = self.path.poses[-5000:]
        self.publisher.publish(self.path)


def main():
    rclpy.init()
    node=ScanMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
