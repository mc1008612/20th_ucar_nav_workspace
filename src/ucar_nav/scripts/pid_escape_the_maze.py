#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
from tf import transformations
import math

# 创建一个Publisher对象，发布到/cmd_vel话题
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

brake_threshold = 0.07
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
    },
    'keep_distance_front_back': {
        'Kp': 0.5,
        'Ki': 0,
        'Kd': 0.015,
        'err': 0.03,  # 误差阈值，单位为米
    },
    'keep_distance_left_right': {
        'Kp': 0.5,
        'Ki': 0,
        'Kd': 0.0,
        'err': 0.03,  # 误差阈值，单位为米
    }
}

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        """
        @Description: 初始化PID控制器
        @param: Kp: 比例增益
        @param: Ki: 积分增益
        @param: Kd: 微分增益
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        """
        @Description: 更新PID控制器并计算输出
        @param: error: 当前误差
        @param: dt: 时间步长
        @return: 控制输出
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        # 输出限幅保持
        if abs(output) < 0.1 and output != 0:
            output = 0.1 if output > 0 else -0.1
        return output

def create_twist_message(x_speed=0, y_speed=0, angular_speed=0):
    """
    @Description: 创建Twist消息对象
    @param: x_speed: x方向速度
    @param: y_speed: y方向速度
    @param: angular_speed: 角速度
    @return: Twist消息对象
    """
    vel_msg = Twist()
    vel_msg.linear.x = x_speed
    vel_msg.linear.y = y_speed
    vel_msg.angular.z = angular_speed
    return vel_msg
    
def get_current_pose():
    """
    @Description: 获取当前机器人姿态
    @return: 当前姿态 (Pose对象)
    """
    return rospy.wait_for_message('now_position', Pose, 50)

def rotate(angle_radians):
    """
    @Description: 旋转指定角度
    @param: angle_radians: 旋转角度 (弧度)
    """
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
    """
    @Description: 控制机器人移动到指定距离，同时保持目标角度
    @NOTE: 使用PID控制器进行距离和角度控制，误差阈值由pid_config['move_forward']['err']或pid_config['move_literal']['err']指定
    @param: distance: 目标移动距离（米）
    @param: target_angle_radians: 目标角度（弧度） default: 0
    @param: direction: 移动方向，'forward' 或 'literal' default: 'forward'
    @param: angle_error_threshold: 角度误差阈值（弧度） default: math.pi/8
    @return: 无
    """
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
    """
    @Description: 对齐到指定角度
    @param: target_angle_radians: 目标角度 (弧度)
    @return: 需要旋转的角度差 (弧度)
    """
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

def pid_control(current_dist, target_dist, pid, dt):
    """
    @Description: PID距离控制核心函数
    @param: current_dist: 当前距离
    @param: target_dist: 目标距离
    @param: pid: PID控制器对象
    @param: dt: 时间步长
    @return: 控制输出, 刹车标志
    """
    error = target_dist - current_dist
    output = pid.update(error, dt)
    brake_flag = abs(error) > brake_threshold  # 设置一个刹车阈值
    return output, brake_flag


# 重命名原有的 keep_distance 方法为 keep_distance_twist
def keep_distance_twist(direction='forward', safe_distance=0.25, threshold=1) -> tuple[bool, Twist]:
    """
    @Description: 保持与障碍物的安全距离
    @param: direction: 方向 ('forward', 'back', 'left', 'right')
    @param: safe_distance: 安全距离
    @param: threshold: 距离阈值
    @return: 是否需要刹车, Twist消息对象
    """
    # 根据方向选择PID控制器参数
    if direction == 'forward' or direction == 'back':
        config = pid_config['keep_distance_front_back']
    elif direction == 'left' or direction == 'right':
        config = pid_config['keep_distance_left_right']
    else:
        rospy.logerr("Invalid direction. Use 'forward', 'back', 'left', or 'right'.")
        return False, create_twist_message()

    # 初始化PID控制器
    pid = PIDController(config['Kp'], config['Ki'], config['Kd'])

    lidar_msg = rospy.wait_for_message('/basic_lidar_info', Float32MultiArray, timeout=10)
    front_distance, back_distance, left_distance, right_distance = lidar_msg.data

    vel_msg = Twist()
    need_brake = False
    t0 = rospy.Time.now().to_sec()

    # 根据方向选择距离
    if direction == 'forward':
        current_distance = front_distance
    elif direction == 'back':
        current_distance = back_distance
    elif direction == 'left':
        current_distance = left_distance
    elif direction == 'right':
        current_distance = right_distance

    # 如果距离超过阈值，不进行控制
    if current_distance > threshold:
        return False, Twist()

    # 计算控制信号
    speed, brake_flag = pid_control(current_distance, safe_distance, pid, rospy.Time.now().to_sec() - t0)

    # 速度限幅（保持在指定的上下限范围内，并保留符号）
    def clamp_speed(speed, min_speed=0.1, max_speed=0.5):
        """
        @Description: 速度限幅
        @param: speed: 当前速度
        @param: min_speed: 最小速度
        @param: max_speed: 最大速度
        @return: 限幅后的速度
        """
        if speed == 0:
            return 0
        # 计算速度的绝对值
        abs_speed = abs(speed)
        # 应用上下限限制
        clamped_abs_speed = max(min(abs_speed, max_speed), min_speed)
        # 恢复原始符号
        return math.copysign(clamped_abs_speed, speed)

    # 根据方向设置速度
    if direction == 'forward':
        vel_msg.linear.x = -clamp_speed(speed)
    elif direction == 'back':
        vel_msg.linear.x = clamp_speed(speed)
    elif direction == 'left':
        vel_msg.linear.y = -clamp_speed(speed)
    elif direction == 'right':
        vel_msg.linear.y = clamp_speed(speed)

    need_brake = brake_flag

    # 达到稳定条件时停止
    if abs(current_distance - safe_distance) < config['err']:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        need_brake = False

    return need_brake, vel_msg

# 新写一个 keep_distance 方法，持续调用 keep_distance_twist，直到达到稳定状态
def keep_distance(direction='forward', safe_distance=0.25, threshold=1):
    """
    @Description: 持续保持与障碍物的安全距离，直到达到稳定状态
    @param: direction: 方向 ('forward', 'back', 'left', 'right')
    @param: safe_distance: 安全距离
    @param: threshold: 距离阈值
    @return: 无
    """
    while not rospy.is_shutdown():
        _, vel_msg = keep_distance_twist(direction, safe_distance, threshold)
        velocity_publisher.publish(vel_msg)
        rospy.loginfo(f"Keep distance {direction}: Safe Distance {safe_distance:.2f}m, Threshold {threshold:.2f}m")
        if vel_msg.linear.x == 0 and vel_msg.linear.y == 0:
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('maze_navigator', anonymous=True)

        # 设置循环频率
        rate = rospy.Rate(10)
        #起点矫正位置
        rotate(align_to_angle(math.radians(0)))
        keep_distance('left',0.25)

        #第一直道
        move(1.5,math.radians(0))

        #第一弯道
        rotate(align_to_angle(math.radians(180)))
        keep_distance('right',0.25)
        
        #第二直道
        rotate(align_to_angle(math.radians(180)))
        move(0.6,target_angle_radians=math.radians(180))

        #识别点
        rospy.sleep(rospy.Duration(3))

        keep_distance('left',0.25)
        rotate(align_to_angle(math.radians(180)))
        move(0.6,target_angle_radians=math.radians(180))
        
        #第二弯道
        keep_distance('left',0.25)
        rotate(align_to_angle(math.radians(0)))
        keep_distance('left',0.25)

        #第三直道
        rotate(align_to_angle(math.radians(0)))
        move(1.5,target_angle_radians=math.radians(0))
        keep_distance('forward',0.3)

        #第三弯道
        keep_distance('right',0.25)
        rotate(align_to_angle(math.radians(180)))
        keep_distance('right',0.25)

        #最终直道
        rotate(align_to_angle(math.radians(180)))
        keep_distance('right',0.25)
        move(1.5,target_angle_radians=math.radians(180))

        #终点矫正位置
        keep_distance('forward',0.3)
        keep_distance('left',0.25)
        rotate(align_to_angle(math.radians(90)))
        keep_distance('left',0.25)

        rospy.set_param('pid_end', 1)
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted")