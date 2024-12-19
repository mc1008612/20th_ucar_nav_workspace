#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@auther: yjy
@fIx: 
封装为类，寻找板子的位置，并发送目标点，将导航分离。
@date: 12/22
"""
"""
@auther:yjy
@Description: 
找板测试程序，寻找板子的位置，并移动到目标位置
@data: 12/15
"""

import rospy
from geometry_msgs.msg import PoseStamped

import os
import sys
sys.path.append((os.path.dirname(os.path.abspath(__file__))))
import lidar_cloud_velocity_pub#引入雷达定位找点模块
import pid_trace#引入pid控制对齐模块


def find_board(plot:bool = False,timeout:bool = True)->PoseStamped:
    """
    @description:找板程序，调用雷达和pid控制器对齐
    @param:plot:是否绘制点云,这会阻塞程序进程 default:False
    @param:timeout:是否超时，如果超时则返回None default:True
    @NOTE:调试时建议开启
    @return:需要到达的目标点位姿
                    如果未找到:返回None
    @raise:None 异常都已经处理
    """
    try:
        """
        @description:引入pid控制器,预先对齐
        """
        rospy.loginfo(">>>in main progress:对齐...")
        pid_controller = pid_trace.PID_BoardTrace_Controller(timeout = timeout)
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
        if plot:
            lidar_cloud_processer.plot_points()
        #等待目标点发布
        if not lidar_cloud_processer.target_board_point == None:
            target_pose = lidar_cloud_processer.target_board_point 
        return target_pose

    except rospy.ROSException:  
        rospy.logerr("雷达点云拟合目标点丢失!")
        return None
    except TimeoutError:
        rospy.logerr("对齐超时!")
        return None
    except KeyboardInterrupt:  
        rospy.logerr("kill progress...")
        return None