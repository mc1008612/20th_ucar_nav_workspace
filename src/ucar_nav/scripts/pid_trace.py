#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
PID Trace test node
@yjy
11/27
"""

import rospy
import yaml
from geometry_msgs.msg import Twist   
import sys

sys.argv.append('debug')
sys.argv.append('no_timeout')
if sys.argv[1] == 'debug':
    DEBUG_MODE = True
else :DEBUG_MODE = False
if sys.argv[2] == 'timeout':
    TIMEOUT = True      
else :TIMEOUT = False   

NONGOAL = 0
GOAL_BUT_NOT_TARGET = 1
FIND_BUT_NOT_ALLIGN = 3
FIND =4

class PID_BoardTrace_Controller(object):
    def __init__(self):
        '''
        从yaml初始化控制器参数
        '''
        try:
            with open('visual_trace_board.yaml', 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr("yaml参数文件不存在!")
        except Exception :
            rospy.logerr("yaml参数文件读取失败!{Exception}")

        #pid velocity param
        self.align_tomid_velocity = config['align_tomid_velocity']
        self.find_target_velocity = config['find_target_velocity']
        #time out check param
        self.timeout_duration_tomid = config['timeout_duration_tomid']
        self.timeout_duration_find_target = config['timeout_duration_find_target']
        '''
        ros服务参数
        '''
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)#cmd_vel发布底盘控制命令
        #ideal guide param 从参数服务器中获取
        self.ideal_tomid = rospy.get_param('ideal_tomid',0.001)  # ~[0,1] 理想中心到目标点直线的距离(归一化)
        self.start_time = rospy.Time.now().to_sec()

        log_info = (
            f"PID Trace node start!\n"
            f"Ideal tomid: {self.ideal_tomid}\n"
            f"Align to mid velocity: {self.align_tomid_velocity}"
        )
        rospy.loginfo(log_info)

    def check_timeout(self,duration):
        """
        :param duration: 超时时间
        :return:null
        :raise: TimeoutException
        """
        if rospy.Time.now().to_sec() - self.start_time > duration:
            raise TimeoutError(f"Timeout! Duration: {duration}")
    def convert_twist(self,x, y,z):
        """
        :param x: x 线速度
        :param y: y 线速度
        :param w: w 角速度
        :return: Twist
        """
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        return twist
    
    def find_target(self):

        rescue_status = rospy.get_param('rescue', NONGOAL)

        if (rescue_status == FIND or 
            rescue_status == FIND_BUT_NOT_ALLIGN):  # 找到目标
                
                self.pub.publish(self.convert_twist(0, 0, 0))
                return True          
        elif (rescue_status == NONGOAL or
            rescue_status == GOAL_BUT_NOT_TARGET):  # 没有找到目标
                
                self.pub.publish(self.convert_twist(0, 0, self.find_target_velocity))
                return False
        else :

            rospy.signal_shutdown("错误的rescue码值")
            exit(1)


    def pub_tomid_velocity(self):
        try:
            tomid = rospy.get_param('rescue_x')
        except rospy.ROSException:
            rospy.logerr("rescue_x参数不存在!")
            exit(1)
        
    # 确保 tomid 在合理范围内
        if not (0 <= tomid <= self.max_tomid):
            rospy.logerr("tomid 值超出范围: {}".format(tomid))
            return False

        #归一化tomid ~norm_tomid~[-1,1]
        norm_tomid = tomid / 0.5

        if abs(norm_tomid) > self.ideal_tomid:#未对齐   
            #pid 控制速度
            pid_velocity_y = - self.align_tomid_velocity * norm_tomid
            self.pub.publish(self.convert_twist(0, pid_velocity_y,0))
            
            if DEBUG_MODE:
                rospy.loginfo(f"pid_velocity_y:{pid_velocity_y},tomid:{tomid}")
            return False
        else :  #对齐
            self.pub.publish(self.convert_twist(0, 0,0))

            if DEBUG_MODE:
                rospy.logerr("已经对齐")
            return True                                                

if __name__ == '__main__' and DEBUG_MODE: 
    try:  
        rospy.init_node('PID_BoardTrace_Controller')
        pid_controller = PID_BoardTrace_Controller()
        
        #寻找目标
        pid_controller.start_time = rospy.Time.now().to_sec()#记录开始时间
        while(not pid_controller.find_target()
              ): 
                if TIMEOUT:pid_controller.check_timeout(pid_controller.timeout_duration_find_target)

        #对齐
        pid_controller.start_time = rospy.Time.now().to_sec()#记录开始时间
        while(not pid_controller.pub_tomid_velocity()
                ):
                if TIMEOUT:pid_controller.check_timeout(pid_controller.timeout_duration_find_target)
        
    except TimeoutError:
        rospy.signal_shutdown("超时!")
        exit(1)
    except KeyboardInterrupt:  
        rospy.signal_shutdown("PID BoardTrace Controller node terminated!")
        exit(2)