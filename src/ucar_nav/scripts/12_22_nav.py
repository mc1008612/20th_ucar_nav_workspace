"""
aurthor: 
    yjy
@Time: 
2024/12/18
@Description:
第一次校内赛导航脚本
"""
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

with open (os.path.join(os.path.dirname(__file__), '12_22_nav_point.yaml'), 'r') as f:
    locations = yaml.load(f)

def success_move_to(location_key:str,max_attempts:int=-1,frame_id:str = 'map')->bool:
    """
    @Description:发布一个目标点，导航到指定目标点
        指定坐标系为map
    @NOTE:特别注意，与find_board中的方法不同，这里是以map坐标系下的目标点
        而find_board中的方法为base_link坐标系下的目标点
    @param:location_key: 目标点名称
    @param:frame_id: 目标点坐标系 defalut:map
    @param:max_attempts: 导航失败重新导航最大尝试次数 default:-1(不进行重试导航)
    @return: 导航成功返回True，失败返回False
    """
    target_pose = locations[location_key]
    attempt = max_attempts

    # 设置目标位姿
    goal = MoveBaseGoal()
    goal.target_pose = target_pose
    goal.target_pose.header.frame_id = frame_id 
    goal.target_pose.header.stamp = rospy.Time.now()

    # 发送目标给move_base并等待结果
    rospy.loginfo(f"正在导航到{location_key}...")
    move_base.send_goal(goal)

    # 等待结果
    state = move_base.get_state()
    while state not in [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.LOST]:
        rospy.sleep(3)
        rospy.loginfo("等待导航结果...")
        state = move_base.get_state()

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo(f"成功导航到{location_key}!")
        return True
    else:
        rospy.logerr(f"导航到{location_key}失败!state:{state},重新导航..." )
        while (attempt >1
                    and not success_move_to(location_key,frame_id,max_attempts-1)):attempt -= 1
        if not attempt == 0:
            return True
        return False#尝试次数耗尽仍然失败
    
def count_points_in_dict(d:dict, prefix='find_board_point_')->int:
    """
    计算字典中符合特定前缀和数字后缀的键的数量
    
    :param d: 字典
    :param prefix: 键的前缀，默认为 'find_board_point_'
    :return: 符合模式的键的数量
    """
    count = 0
    for key in d.keys():
        if key.startswith(prefix):
            suffix = key[len(prefix):]
            if suffix.isdigit():
                count += 1
    return count

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('12_22_nav_py', anonymous=False)
        # 创建ActionClient
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器启动...")  
        move_base.wait_for_server()

        """
        @开始逐个点导航
        """
        if success_move_to('terrorist_point',3):#前往恐怖分子点成功
            rospy.loginfo("前往恐怖分子点成功")
        else:
            rospy.logerr("前往恐怖分子点失败，正在返航")
            if success_move_to('start_point'):
                rospy.loginfo("返航成功")
                success_move_to('terrorist_point')#重新前往恐怖分子点       
        rospy.set_param('take_photo', 1)#启动拍照
        while rospy.get_param('take_photo') ==1:pass#阻塞等待识别结果

        if success_move_to('first_aid_kit',3):#前往恐怖分子点成功
            rospy.loginfo("前往急救包点成功")
        else:
            rospy.logerr("前往急救包失败，正在返航")
            if success_move_to('start_point'):
                rospy.loginfo("返航成功")
                success_move_to('terrorist_point')#重新前往急救包点       
        rospy.set_param('take_photo', 1)#启动拍照
        while rospy.get_param('take_photo') ==1:pass#阻塞等待识别结果
        
        """
        到各个目标点找板子
        """
        find_board_point_len = count_points_in_dict(locations)
        board_park_point = PoseStamped()#停车识别点

        for i in range(find_board_point_len):
            if success_move_to(f'find_board_point_{i}'):#前往目标点成功
                rospy.loginfo(f"前往{i}号板子成功")
                board_park_point = find_board.find_board()
                if not board_park_point ==None:#成功获得板子泊车点坐标，则进行泊车到
                    success_move_to(,board_park_point)


    except rospy.ROSInterruptException:
        rospy.loginfo("主导航脚本已退出")

