import os

import rospy
import yaml
from geometry_msgs.msg import Twist

#临时状态码，下次和导航配合起来后删除
NONGOAL = 0
GOAL_BUT_NOT_TARGET = 1
FIND_BUT_NOT_ALLIGN = 3
FIND = 4


class PID_BoardTrace_Controller:
    """
    @description: 自转动寻找目标点，视觉pid对正目标点，并控制底盘移动到目标点
    @return: 发布'nav_status'参数判断状态
    """
    def __init__(self,config_file_path=os.path.join(os.path.dirname(__file__), 'visual_trace_board.yaml'), timeout=False):
        """
       @description: 自转动寻找目标点，视觉pid对正目标点，并控制底盘移动到目标点
        @param: config_file_path: 配置文件路径
        @param: timeout: 是否开启超时退出
        @return: 发布'nav_status'参数判断状态
        NOTE: nav_status:
        用于发布导航状态的参数
        [ERROR,     LIDAR_FOUND,    LIDAR_FIND,   #本文件中相关状态
        START
        FOUND,       FOUND_BUT_NOT_ALIGN,
         ALIGN,      NONGOAL,    GOAL_BUT_NOT_TARGET
        #pid_trace.py中发布状态]
        发布:FOUND_BUT_NOT_ALIGN 发布'nav_status'为'FOUND_BUT_NOT_ALIGN'，表示已经找到目标点，但是没有对齐
                ALIGN 发布'nav_status'为'ALIGN'，表示已经对齐
                LIDAR_FIND 发布'nav_status'为'LIDAR_FIND'，表示开始通过激光雷达寻找目标点
                ERROR 发布'nav_status'为'ERROR'，表示程序出错
        接受:START:接受视觉节点置'nav_status'为START，表示开始导航
        """
        try:
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr("yaml参数文件不存在!")
            raise
        except Exception as e:
            rospy.logerr(f"yaml参数文件读取失败! {e}")
            raise

        # PID速度控制参数
        self.find_target_velocity = config['find_target_velocity']
        pid_param = config['tomid_pid_param']
        self.kp_tomid = pid_param[0]  # 添加 kp 参数
        self.ki_tomid = pid_param[1]  # 添加 ki 参数
        self.kd_tomid = pid_param[2]  # 添加 kd 参数
        # 超时退出参数
        self.timeout_duration_tomid = config['timeout_duration_tomid']
        self.timeout_duration_find_target = config['timeout_duration_find_target']
        # tomid刷新率
        self.tomid_fresh_period = config['tomid_fresh_period']

        '''
        ros服务参数
        '''
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel发布底盘控制命令
        # ideal guide param 从参数服务器中获取
        self.ideal_tomid = rospy.get_param('ideal_tomid', 0.05)  # ~[0,1] 理想中心到目标点直线的距离(归一化)
        # 任务开始时间
        self.start_time_stamp = None

        # 临时变量
        self.integral = 0.0  # 积分项
        self.previous_error = 0.0  # 上一次的误差

        log_info = (
            f"PID Trace node run!\n"
            f"Ideal tomid: {self.ideal_tomid}\n"
            f"tomid_fresh_peroid:{self.tomid_fresh_period}\n"
            f"p&i&d:{self.kp_tomid}\t,{self.ki_tomid},\t{self.kd_tomid}"
        )
        rospy.loginfo(log_info)

        self.TIMEOUT = timeout
        self.timer:rospy.Timer = None 

    def check_timeout(self, duration):
        """
        :param duration: 超时时间
        :return: null
        :raise: TimeoutException
        """
        if rospy.Time.now().to_sec() - self.start_time_stamp > duration:
            raise TimeoutError(f"Timeout! Duration: {duration}")

    def convert_twist(self, x, y, z):
        """
        :param x: x 线速度
        :param y: y 线速度
        :param z: z 角速度
        :return: Twist
        """
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        return twist

    def find_target(self):
        rescue_status = rospy.get_param('rescue', NONGOAL)
        #nav_status = rospy.get_param('nav_status')

        if (rescue_status == FIND or
                rescue_status == FIND_BUT_NOT_ALLIGN):  # 找到目标
            self.pub.publish(self.convert_twist(0, 0, 0))
            return True
        elif (rescue_status == NONGOAL or
              rescue_status == GOAL_BUT_NOT_TARGET):  # 没有找到目标
            self.pub.publish(self.convert_twist(0, 0, self.find_target_velocity))
            return False
        else:
            rospy.logerr("错误的rescue码值")
            raise ValueError

    def tomid_sample_timer_callback(self, event):
        """
        @Describe: 定时器的回调函数,控制每个采样周期位置移动的增量:
        delta_y = v* t
        使用参数:P控制:v = Kp*tomid
        定时采样tomid,更新pid_output_velocity_y
        在采样后创建一段时间,发布速度
        如果速度小于0.1.则控制发布的时间，达到控制增量的目的
        需要注意：
        采样间隔需要大于tomid更新时间间隔
        @Returns: null
        """
        # 获取当前 self.tomid 值
        try:
            tomid = rospy.get_param('rescue_x')
            self.rescue_status = rospy.get_param('rescue')
            if self.rescue_status == NONGOAL:
                raise Exception
            if self.rescue_status == FIND:
                rospy.logwarn('had FIND')
                self.stop()#had FIND
                return
        except rospy.ROSException:
            rospy.logerr("rescue_x 参数不存在!") # 程序出口:参数不存在
            return
        except Exception:
            rospy.logerr("LOST")
            #重新启用程序
            if self.timer.is_alive:
                self.timer.shutdown()
            rospy.logerr("正在重启程序...")
            self.run()

        # 确保 self.tomid 在合理范围内 (-0.5 到 0.5)
        if not (-0.5 <= tomid <= 0.5):
            rospy.logerr(f"self.tomid 值超出范围: {tomid}")
            return  # 程序出口:tomid 超出范围

        # 归一化 self.tomid 到 [-1, 1]
        normalized_tomid = tomid / 0.5  # tomid合理，使用值进行控制更新

        # 检查是否对齐&FIND
        if abs(normalized_tomid) <= self.ideal_tomid:
            self.pub.publish(self.convert_twist(0, 0, 0))  # 停车
            # 重置积分项和上一次的误差
            self.integral = 0.0
            self.previous_error = 0.0
            rospy.set_param('nav_status', 'ALIGN')#设置状态参数为对齐
            rospy.set_param('nav_status','LIDAR_FIND')
            self.stop()# 程序唯一出口:对齐
            return

        # 未对齐：                                                                                                   对delta_y进行pid

        # 计算PID 控制速度
        error = normalized_tomid
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        pid_output_velocity_y = -(self.kp_tomid * error +
                                  self.ki_tomid * self.integral + self.kd_tomid * derivative)

        # 针对小于0.1的pid输出，做时间pid补偿
        if abs(pid_output_velocity_y) < 0.1:
            """
            # 只在速度小于0.1时,做时间控制补偿增量输出
            # param: pid_output_velocity_y_time
            """
            pid_output_velocity_y_time = (abs(pid_output_velocity_y) / 0.1) * self.tomid_fresh_period/1.5
            # 将发布时间速度置回+-0.1
            if not pid_output_velocity_y ==0:
                pid_output_velocity_y = 0.1 * pid_output_velocity_y / abs(pid_output_velocity_y)
            else:
                pid_output_velocity_y  = 0
                rospy.logerr('aligned')
        else:  # pid速度大于0.1的情况:跑满整个间隔
            pid_output_velocity_y_time = self.tomid_fresh_period

        rospy.loginfo(f"pid_output_velocity_y: {pid_output_velocity_y}, "
                        f"time:{pid_output_velocity_y_time}, \t tomid: {tomid}")

        # 创建一段时间通过速度控制增量
        time_stamp = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time_stamp < pid_output_velocity_y_time:
            self.pub.publish(self.convert_twist(0, pid_output_velocity_y, 0))

    def run(self):
        """
        @Description: 程序入口函数
        @Returns: null
        @raise:TimeoutError 
                ValueError
        """
        self.start_time_stamp = rospy.Time.now().to_sec()  # 记录开始时间

        # 阻塞寻找目标
        while not self.find_target():
            if self.TIMEOUT:
                self.check_timeout(self.timeout_duration_find_target)

        # 对齐
        self.start_time_stamp = rospy.Time.now().to_sec()  # 记录开始时间
        self.timer = rospy.Timer(rospy.Duration(self.tomid_fresh_period), self.tomid_sample_timer_callback)

    def stop(self):
        """
        @Description: 程序退出函数,关闭定时器
        @Returns: null
        """
        if self.timer.is_alive:
            self.timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('PID_BoardTrace_Controller')
    controller = PID_BoardTrace_Controller()
    controller.run()
