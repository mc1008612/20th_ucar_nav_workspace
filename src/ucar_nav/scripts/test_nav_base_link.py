#!/usr/bin/env python
# -*- coding: utf-8 -*-


#movebase
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
import yaml

import os
import sys
sys.path.append((os.path.dirname(os.path.abspath(__file__))))
import find_board

locations = []
def success_move_to(location_key:str=None, pose:PoseStamped=None, max_attempts:int = -1, frame_id:str='map') -> bool:
    """
    @Description: 发布一个目标点，导航到指定目标点
        指定坐标系为map
    @NOTE: 特别注意，与find_board中的方法不同，这里是以map坐标系下的目标点
        而find_board中的方法为base_link坐标系下的目标点
    @param: location_key: 目标点名称 (可选)
    @param: pose: 目标点 (PoseStamped对象) (可选)
    @param: frame_id: 目标点坐标系 default: map
    @param: max_attempts: 导航失败重新导航最大尝试次数 default: -1 (不进行重试导航)
    @return: 导航成功返回True，失败返回False
    """
    if location_key is None and pose is None:
        rospy.logerr("必须提供location_key或goal")
        return False

    goal = MoveBaseGoal()
    location_poseStamped = PoseStamped()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()

    if location_key is not None:#location_key
        location_poseStamped = locations[location_key]
    else:#pose
        location_poseStamped = pose

    goal.target_pose.pose.position = location_poseStamped.pose.position
    goal.target_pose.pose.orientation = location_poseStamped.pose.orientation

    attempt = max_attempts
    # 发送目标给move_base并等待结果
    rospy.loginfo(f"正在导航到目标点...")
    move_base.send_goal(goal)

    # 等待结果
    state = move_base.get_state()
    while state not in [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
        rospy.sleep(3)
        rospy.loginfo("等待导航结果...")
        state = move_base.get_state()

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo(f"成功导航到目标点!")
        return True
    else:
        rospy.logerr(f"导航到目标点失败! state: {state}, 重新导航...")
        while (attempt > 1
            and not success_move_to(location_key=location_key, pose = pose, max_attempts=-1)):
            attempt -= 1
        if not attempt == 0:
            return True
        return False  # 尝试次数耗尽仍然失败
if __name__ == '__main__':
    try:
        """
        找板子
        """
        # 初始化ROS节点
        rospy.init_node('test_nav_py', anonymous=True)
        # 创建ActionClient
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器启动...")  
        move_base.wait_for_server()

        board_park_point = find_board.find_board(plot=True,timeout=False)
        if not board_park_point ==None:#成功获得板子泊车点坐标，则进行泊车到
            success_move_to(pose = board_park_point,max_attempts=3,frame_id='base_link')
    except KeyboardInterrupt:
        sys.exit(0)