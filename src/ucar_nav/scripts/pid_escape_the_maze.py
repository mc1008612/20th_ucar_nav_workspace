#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from tf import transformations
import math

# PID控制器参数配置（包含误差阈值）
pid_config = {
    'move_forward': {
        'Kp': 0.4, 
        'Ki': 0, 
        'Kd': 0.015,
        'err': 0.03  # 新增移动误差阈值
    },
    'move_literal': {
        'Kp': 0.3, 
        'Ki': 0, 
        'Kd': 0.01,
        'err': 0.03  # 新增移动误差阈值
    },
    'rotate': {
        'Kp': 0.8,
        'Ki': 0,
        'Kd': 0.01,
        'err': 0.01  # 新增旋转误差阈值
    }
}

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        # 输出限幅保持
        if abs(output) < 0.1:
            output = 0.1 if output > 0 else -0.1
        return output

def create_twist_message(x_speed=0, y_speed=0, angular_speed=0):
    vel_msg = Twist()
    vel_msg.linear.x = x_speed
    vel_msg.linear.y = y_speed
    vel_msg.angular.z = angular_speed
    return vel_msg

def get_current_pose():
    return rospy.wait_for_message('now_position', Pose, 50)

def rotate(angle_radians):
    config = pid_config['rotate']
    pid = PIDController(config['Kp'], config['Ki'], config['Kd'])

    initial_pose = get_current_pose()
    if not initial_pose:
        rospy.logerr("Failed to get initial pose")
        return

    start_angle = transformations.euler_from_quaternion([
        initial_pose.orientation.x,
        initial_pose.orientation.y,
        initial_pose.orientation.z,
        initial_pose.orientation.w
    ])[2]

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        current_pose = get_current_pose()
        if not current_pose:
            rospy.logerr("Failed to get current pose")
            break

        current_angle = transformations.euler_from_quaternion([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ])[2]

        error = angle_radians - (current_angle - start_angle)
        error = (error + math.pi) % (2 * math.pi) - math.pi  # 规范化角度误差

        dt = rospy.Time.now().to_sec() - t0
        t0 = rospy.Time.now().to_sec()

        angular_speed = pid.update(error, dt)
        vel_msg = create_twist_message(angular_speed=angular_speed)
        velocity_publisher.publish(vel_msg)

        # 修改日志输出为角度（度数）
        rospy.loginfo(f"Rotate: Current {math.degrees(current_angle):.2f}deg, "
                     f"Target {math.degrees(start_angle + angle_radians):.2f}deg, "
                     f"Error {math.degrees(error):.2f}deg, "
                     f"Speed {angular_speed:.2f}rad/s")

        if abs(error) < config['err']:
            break
        rate.sleep()

    create_twist_message(angular_speed=0)
    rospy.loginfo("Rotation completed")

def move(distance,  target_angle_radians=0,direction='forward', angle_error_threshold=math.pi/8):
    # 根据方向选择PID控制器参数
    if direction == 'forward':
        config = pid_config['move_forward']
    elif direction == 'literal':
        config = pid_config['move_literal']
    else:
        rospy.logerr("Invalid direction. Use 'forward' or 'literal'.")
        return

    pid_move = PIDController(config['Kp'], config['Ki'], config['Kd'])
    pid_rotate = PIDController(pid_config['rotate']['Kp'], pid_config['rotate']['Ki'], pid_config['rotate']['Kd'])

    # 获取初始位姿
    initial_pose = get_current_pose()
    if not initial_pose:
        rospy.logerr("Failed to get initial pose")
        return

    # 记录初始位置
    initial_x, initial_y = initial_pose.position.x, initial_pose.position.y
    t0 = rospy.Time.now().to_sec()

    # 循环发布速度消息，直到移动指定距离
    while not rospy.is_shutdown():
        current_pose = get_current_pose()
        if not current_pose:
            rospy.logerr("Failed to get current pose")
            break

        # 记录当前位置
        current_x, current_y = current_pose.position.x, current_pose.position.y

        # 计算已移动的距离
        current_distance = math.hypot(current_x - initial_x, current_y - initial_y)
        # 根据distance的符号调整current_distance的符号
        if distance < 0:
            current_distance = -current_distance

        # 计算误差
        error_distance = distance - current_distance

        # 计算时间差
        dt = rospy.Time.now().to_sec() - t0
        t0 = rospy.Time.now().to_sec()

        # 计算控制信号
        speed = pid_move.update(error_distance, dt)

        # 计算角度误差
        current_angle = transformations.euler_from_quaternion([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ])[2]
        angle_to_rotate = target_angle_radians - current_angle  # 使用传入的目标角度
        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi  # 规范化角度误差
        angular_speed = pid_rotate.update(angle_to_rotate, dt)

        # 根据角度误差调整速度
        angle_error_factor = 1 - abs(angle_to_rotate) / angle_error_threshold
        angle_error_factor = max(angle_error_factor, 0)  # 确保angle_error_factor不小于0
        adjusted_speed = speed * angle_error_factor

        # 创建Twist消息对象
        if direction == 'forward':
            vel_msg = create_twist_message(x_speed=adjusted_speed, angular_speed=angular_speed)
        elif direction == 'literal':
            vel_msg = create_twist_message(y_speed=adjusted_speed, angular_speed=angular_speed)

        # 发布速度消息
        velocity_publisher.publish(vel_msg)

        # 修改日志输出为角度（度数）
        rospy.loginfo(f"Move {direction}: Current {current_distance:.2f}m, "
                     f"Target {distance:.2f}m, "
                     f"Error {error_distance:.2f}m, "
                     f"Speed {adjusted_speed:.2f}m/s, "
                     f"Angle Error {math.degrees(angle_to_rotate):.2f}deg, "
                     f"Angular Speed {angular_speed:.2f}rad/s")

        # 检查是否达到目标距离
        if abs(error_distance) < config['err']:
            break

        # 按照设置的频率休眠
        rate.sleep()

    # 停止移动
    vel_msg = create_twist_message()
    velocity_publisher.publish(vel_msg)
    rospy.loginfo(f"Move {direction} completed")

def align_to_angle(target_angle_radians):
    current_pose = get_current_pose()
    if not current_pose:
        rospy.logerr("Failed to get current pose")
        return

    current_angle = transformations.euler_from_quaternion([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ])[2]

    # 计算需要旋转的角度差
    angle_to_rotate = target_angle_radians - current_angle
    angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi  # 规范化角度误差

    return angle_to_rotate

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('maze_navigator', anonymous=True)

        # 创建一个Publisher对象，发布到/cmd_vel话题
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 设置循环频率
        rate = rospy.Rate(10)
        
        #左正右负 前正后负

        move(1.5,target_angle_radians=math.radians(0))

        rotate(math.radians(180))
        move(-0.5,direction = 'literal',target_angle_radians=math.radians(180))
        move(1.2,target_angle_radians=math.radians(180))

        rotate(math.radians(180))
        move(0.5,direction = 'literal',target_angle_radians=math.radians(0))
        move(1.7,target_angle_radians=math.radians(0))

        rotate(math.radians(180))
        move(-0.5,direction = 'literal',target_angle_radians=math.radians(180))
        move(2,target_angle_radians=math.radians(180))

        rospy.set_param('pid_end', 1)
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted")