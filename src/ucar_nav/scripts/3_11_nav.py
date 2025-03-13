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
    #'start_point',
    'select_point':Pose(Point(1.1096277236938477,3.2154581546783447 ,0.0), Quaternion(0.0,0.0,0.7039566779415104,0.7102429130808365)),
    #点的顺序为头朝向地图北，逆时针旋转
    
    # 'pickup_point-1',
    # 'pickup_point-2',
    # 'pickup_point-3',
    
    #'cross_bridge_point',
    'traffic_light_point':Pose(Point(3.031625747680664,3.807620048522949 ,0.0), Quaternion(0.0,0.0,0.6981053788795045,0.7159950279013838)),
    'cross_road_point-1':Pose(Point(2.6082823276519775,3.4113075733184814 ,0.0), Quaternion(0.0,0.0,-0.7012479374651338,0.7129174778337923)),
    'cross_road_point-2':Pose(Point(4.5017476081848145,3.2250804901123047 ,0.0), Quaternion(0.0,0.0,-0.6940040541053553,0.7199710917011398))
    }


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
        ideal_board_distance = 0.25
        ideal_distance_for_traffic_light = 0.5

        # 初始化ROS节点
        rospy.init_node('3_11_nav_py', anonymous=False)
        # 创建ActionClient
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器启动...")  
        #创建速度控制发布者
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        move_base.wait_for_server()

        while(rospy.get_param('pid_end',0)==0):pass

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
        rospy.set_param('take_photo', 1)
        angles = [math.radians(90), math.radians(180), math.radians(270)]
        timeout_duration = rospy.Duration(3.0)

        for angle in angles: 
            pid.rotate(pid.align_to_angle(angle))
            find_flag = False
            start_time = rospy.Time.now()

            while rospy.get_param('object_state', 0) == 0:
                if rospy.Time.now() - start_time > timeout_duration:
                    rospy.logwarn(f"等待object_state超时，跳出循环，当前角度: {angle}rad")
                    continue
                if rospy.get_param('object_state', 0) == 1:
                    rospy.loginfo(f"未找到目标，继续旋转，当前角度: {angle}rad")
                if rospy.get_param('object_state', 0) in [3, 4]:
                    rospy.loginfo(f"找到目标，跳出循环，当前角度: {angle}rad")
                    find_flag = True
                    break
                rospy.sleep(0.1)  # 添加一个小的睡眠以避免CPU占用过高

            if find_flag:
                break
        else :
            rospy.logerr("未找到目标")

        keep_distance = pid.keep_distance('forward',ideal_board_distance)
        """
        到检测红绿灯点
        """
        #方案1直接导航
        success_move_to(location_key='traffic_light_point',max_attempts = 3) 
        
        #方案2过桥
        # success_move_to(location_key='cross_bridge_point',max_attempts=2)
        # pid.move('forward',1)
        # success_move_to(location_key='traffic_light_point',max_attempts=2)
        
        """
        识别红绿灯
        """
        pid.rotate(pid.align_to_angle(math.radians(90)))
        pid.keep_distance('forward',ideal_distance_for_traffic_light)    
        rospy.set_param('take_photo', 1)#启动拍照
        while rospy.get_param('take_photo',0) ==1:pass#阻塞等待识别结果
        if rospy.get_param('able_to_go', 0)==1:
            rospy.loginfo("前往交叉口1")
            success_move_to('cross_road_point-1')
        else:
            rospy.loginfo("前往交叉口2")
            success_move_to('cross_road_point-2')
        
        """
        巡线点
        """
        rospy.set_param('start_trace',1)
        ###等待巡线终止条件

        rospy.loginfo("导航结束")
        rospy.set_param('stop_flag',1)
        


    except rospy.ROSInterruptException:
        rospy.loginfo("主导航脚本已退出")

