#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@auther:yjy
@Description: 
找板测试程序，寻找板子的位置，并移动到目标位置
@data: 12/15
"""

#movebase
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

import os
import sys
sys.path.append((os.path.dirname(os.path.abspath(__file__))))
import lidar_cloud_velocity_pub#引入雷达定位找点模块
import pid_trace#引入pid控制对齐模块


def move_to_pose(target_pose):
    # 创建ActionClient
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # 等待move_base服务器启动
    rospy.loginfo("等待move_base服务器启动...")  
    move_base.wait_for_server()

    # 设置目标位姿
    goal = MoveBaseGoal()
    goal.target_pose = target_pose
    goal.target_pose.header.frame_id = 'base_link'  
    goal.target_pose.header.stamp = rospy.Time.now()

    # 发送目标给move_base并等待结果
    rospy.loginfo("正在导航到目标板子前...")
    move_base.send_goal(goal)

    # 等待结果
    state = move_base.get_state()
    while state not in [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
        rospy.sleep(3)
        rospy.loginfo("Waiting for result...")
        state = move_base.get_state()

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("成功到目标板子前!")
    else:
        rospy.loginfo("导航到目标板子前失败！" % state)

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('find_board_py', anonymous=True)
        """
        @description:引入pid控制器,预先对齐
        """
        rospy.loginfo(">>>in main progress:对齐...")
        pid_controller = pid_trace.PID_BoardTrace_Controller(timeout=False)
        pid_controller.run()
        #检查进程退出情况
        while pid_controller.timer.is_alive():
            rospy.logwarn("等待pid控制器执行完毕...")
            rospy.sleep(5)
        """
        @description:雷达点云拟合 移动到目标点
        """ 
        rospy.loginfo(">>>in main progress:雷达点云拟合...")
        lidar_cloud_processer = lidar_cloud_velocity_pub.LidarCloudVelocityPub(debug_mode=False)
        lidar_cloud_processer.run()
        lidar_cloud_processer.plot_points()
        """
        @description:订阅雷达点云拟合出的目标点与姿态方向，移动到目标点
        """
        rospy.loginfo(">>>in main progress:导航到目标点...")
        target_pose = PoseStamped()
        #等待目标点发布
        if not lidar_cloud_processer.target_board_point == None:
            target_pose = lidar_cloud_processer.target_board_point 
        move_to_pose(target_pose)
        rospy.loginfo(">>>in main progress:已到达板子前...")

    except rospy.ROSException:  
        rospy.logerr("雷达点云拟合目标点丢失!")
        exit(1)
    except TimeoutError:
        rospy.signal_shutdown("对齐超时!")
        exit(1)
    except KeyboardInterrupt:  
        rospy.signal_shutdown("kill progress...")
        exit(2)