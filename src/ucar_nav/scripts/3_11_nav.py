"""
aurthor: 
    yjy
@Time: 
2024/3/11
@Description:
第二周进度检查导航脚本
"""
#!/usr/bin/env python
# -*- coding: utf-8 -*-


#movebase
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped,Pose,Point, Quaternion,Twist
from std_msgs.msg import Float32MultiArray
from actionlib_msgs.msg import GoalStatus

import os
import sys
sys.path.append((os.path.dirname(os.path.abspath(__file__))))
import pid_escape_the_maze as pid
from tf import transformations
import math

locations = {
    'start_point',
    'select_point',
    #点的顺序为头朝向地图北，逆时针旋转
    'pickup_point-1',
    'pickup_point-2',
    'pickup_point-3',
    'cross_bridge_point',
    'traffic_light_point',
    'cross_road_point-1',
    'cross_road_point-2'
    ''}


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
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()

    if location_key is not None:#location_key
        goal.target_pose.pose = locations[location_key]
    else:#pose
        goal.target_pose.pose = pose.pose


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
        @param:
        """
        ideal_board_distance = 0.2
        ideal_distance_for_traffic_light = 0.5

        # 初始化ROS节点
        rospy.init_node('3_9_nav_py', anonymous=False)
        # 创建ActionClient
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器启动...")  
        #创建速度控制发布者
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        move_base.wait_for_server()

        """
        @开始逐个点导航
        """
        """
        前往选货点
        """
        success_move_to(location_key='select_point',max_attempts = 3)

        """
        选货
        """
        rotate_angular_speed = 0.5
        rospy.set_param('take_photo',1)
        while rospy.get_param('take_photo') ==1:
            vel_pub(pid.create_twist_message(0,0,rotate_angular_speed))
        # 获取当前位置，根据粗略位置大概确定方向
        now_position = rospy.wait_for_message('now_position',Pose,5)
        now_angular = transformations.euler_from_quaternion(now_position.orientation)[2]
        # 计算now_angular与90, 180, 270的差值
        angles = [90, 180, 270]
        differences = [abs(now_angular - angle) for angle in angles]
        # 找到差值最小的索引
        min_index = differences.index(min(differences))
        # 将now_angular修正为差值最小的那个值
        #同时角度转弧度
        now_angular = math.radians(angles[min_index])
        #矫正位置
        pid.rotate(pid.align_to_angle(now_angular))
        pid.move(0.5,now_angular)
        pid.keep_distance(ideal_board_distance)

        """
        到检测红绿灯点
        """
        success_move_to(location_key='traffic_light_point',max_attempts = 3) 
        pid.rotate(pid.align_to_angle(math.radians(90)))
        pid.keep_distance(ideal_distance_for_traffic_light)    
        rospy.set_param('take_photo', 1)#启动拍照
        while rospy.get_param('take_photo') ==1:pass#阻塞等待识别结果
        
        """
        决策，前往哪一个路口
        """
        

        rospy.loginfo("导航结束")
        rospy.set_param('stop_flag',1)
        


    except rospy.ROSInterruptException:
        rospy.loginfo("主导航脚本已退出")

