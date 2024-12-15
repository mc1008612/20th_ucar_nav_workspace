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
        pid_controller = pid_trace.PID_BoardTrace_Controller()
        #寻找目标
        pid_controller.start_time = rospy.Time.now().to_sec()  # 记录开始时间
        while not pid_controller.find_target():
                pid_controller.check_timeout(pid_controller.timeout_duration_find_target)

        # 对齐
        pid_controller.start_time = rospy.Time.now().to_sec()  # 记录开始时间
        try:
            rospy.Timer(rospy.Duration(pid_controller.tomid_fresh_period), pid_controller.tomid_sample_timer_callback)
            while(not pid_controller.check_timeout(pid_controller.timeout_duration_tomid)):
                pass
        except KeyboardInterrupt:
            #正常退出    
            """
            @description:雷达点云拟合 移动到目标点
            """ 
            lidar_cloud_velocity_pub. main()
        rospy.spin()  # 可选，用于保持节点运行并处理回调等（在这个简单脚本中可能不是必需的）
    except TimeoutError:
        rospy.signal_shutdown("超时!")
        exit(1)
    except KeyboardInterrupt:  
        rospy.signal_shutdown("PID BoardTrace Controller node terminated!")
        exit(2)