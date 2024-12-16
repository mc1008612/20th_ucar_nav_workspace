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
from geometry_msgs.msg import PoseStamped,Point,Quaternion
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
    goal.target_pose.header.frame_id = 'map'  
    goal.target_pose.header.stamp = rospy.Time.now()

    # 发送目标给move_base并等待结果
    rospy.loginfo("正在导航到目标板子前...")
    move_base.send_goal(goal)

    # 等待结果
    state = move_base.get_state()
    while state not in [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
        rospy.sleep(1)
        state = move_base.get_state()

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("The base moved to the goal pose.")
    else:
        rospy.loginfo("The base failed to move to the goal pose. State: %s" % state)

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('find_board_py', anonymous=True)
        """
        @description:引入pid控制器,预先对齐
        """
        pid_controller = pid_trace.PID_BoardTrace_Controller(debug_mode=False,timeout=True)
        pid_controller.run()
        """
        @description:雷达点云拟合 移动到目标点
        """ 
        lidar_cloud_processer = lidar_cloud_velocity_pub.LidarCloudVelocityPub(debug_mode=False,plot=False)
        lidar_cloud_processer.run()
        """
        @description:订阅雷达点云拟合出的目标点与姿态方向，移动到目标点
        """
        target_pose = PoseStamped()
        #等待目标点发布
        target_pose = rospy.wait_for_message("target_board_point", PoseStamped, timeout=5)
        move_to_pose(target_pose)
        rospy.spin()
    except rospy.ROSException:  
        rospy.signal_shutdown("雷达点云拟合目标点丢失!")
        exit(1)
    except TimeoutError:
        rospy.signal_shutdown("对齐超时!")
        exit(1)
    except KeyboardInterrupt:  
        rospy.signal_shutdown("PID BoardTrace Controller node terminated!")
        exit(2)