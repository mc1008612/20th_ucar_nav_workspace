#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
import logging

# 配置日志
logging.basicConfig(level=logging.ERROR)

"""
作者：yjy
日期：12/1
功能：获取当前位姿
描述：根据tf获取/map到/base_link的位姿数据,发布/base_link坐标系tf
"""

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('tf_listener')
    
    # 创建一个Buffer对象和一个TransformListener对象，用于缓存和监听tf变换
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 创建一个Publisher对象，用于发布位姿数据
    pose_publisher = rospy.Publisher('now_position', Pose, queue_size=1)
    
    # 设置循环频率为20Hz
    rate = rospy.Rate(20.0)
    
    # 主循环，尝试获取并发布位姿数据
    while not rospy.is_shutdown():
        try:
            # 等待并获取 /map 到 /base_link 的tf变换
            transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            
            # 将获取到的位姿数据输出到控制台
            current_pose = Pose()
            current_pose.position.x = transform.transform.translation.x
            current_pose.position.y = transform.transform.translation.y
            current_pose.position.z = transform.transform.translation.z
            current_pose.orientation.x = transform.transform.rotation.x
            current_pose.orientation.y = transform.transform.rotation.y
            current_pose.orientation.z = transform.transform.rotation.z
            current_pose.orientation.w = transform.transform.rotation.w
            
            # 发布位姿数据
            pose_publisher.publish(current_pose)
        except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
            # 记录异常信息
            logging.error(f"Error in lookup_transform: {e}")
        
        # 按照设定的频率休眠
        rate.sleep()