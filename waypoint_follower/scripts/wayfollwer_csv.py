#! /usr/bin/env python3
import rclpy
import os
import pandas as pd
import time
from robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

# data= pd.read_csv(os.path.join(get_package_share_directory("waypoint_follower"), "scripts", "robot_position.csv"))
data = pd.read_csv(r'/home/amr/ros_ws/src/tortoisebot_pro_max-ros2/waypoint_follower/scripts/robot_position.csv')

store_len = int(len(data.index))
rclpy.init()
for i in range(store_len):
    
    navigator= BasicNavigator()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = data.x[i]
    goal_pose.pose.position.y = data.y[i]
    goal_pose.pose.position.z = data.z[i]
    goal_pose.pose.orientation.x = data.qx[i]
    goal_pose.pose.orientation.y = data.qy[i]
    goal_pose.pose.orientation.z = data.qz[i]
    goal_pose.pose.orientation.w = data.qw[i]
    navigator.goToPose(goal_pose)
    
    while not navigator.isNavComplete():
        print("Moving towards goal",i+1)
    print("Goal Reached!")
    # delay = 15.0  # Delay between each goal in seconds
    time.sleep(15)  # Delay for 15 second (adjust as needed)
print("Task Completed")
exit(0)
