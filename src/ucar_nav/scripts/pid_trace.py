import rospy
import yaml
from geometry_msgs.msg import Twist   
import sys

NONGOAL = 0
"""@const:NONGOAL:未找到目标"""
GOAL_BUT_NOT_TARGET = 1
"""@const:GOAL_BUT_NOT_TARGET:找到目标，但未对齐"""
FIND_BUT_NOT_ALLIGN = 3
"""@const:FIND_BUT_NOT_ALLIGN:找到目标，但未对齐"""
FIND = 4 
"""@const:FIND:找到目标，对齐"""



"""
@Description:配置DEBUG与TIMEOUT测试模式
"""

if len(sys.argv) < 2:
    sys.argv.append('debug')  # 默认为debug模式
if len(sys.argv) < 3:
    sys.argv.append('no_timeout')  # 默认不开启timeout模式

arg1 = sys.argv[1]
arg2 = sys.argv[2]

valid_modes = {'debug', 'no_debug'}
valid_timeouts = {'timeout', 'no_timeout'}

if arg1 not in valid_modes:
    raise ValueError(f"无效的参数 {arg1}, 请使用 'debug' 或 'no_debug'")

if arg2 not in valid_timeouts:
    raise ValueError(f"无效的参数 {arg2}, 请使用 'timeout' 或 'no_timeout'")

DEBUG_MODE = arg1 == 'debug' 
TIMEOUT = arg2 == 'timeout'

class PID_BoardTrace_Controller(object):     
    def __init__(self):
        """
        @Description:PID_BoardTrace_Controller 节点
        @Fuction: 创建pid控制器,实现对目标板的位置追踪
        @Author:   yjy
        @Version:  1.0
        @Modify:   2024.12.15
        """
        try:
            with open('/home/ucar/ucar_ws/src/ucar_nav/scripts/visual_trace_board.yaml', 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr("yaml参数文件不存在!")
        except Exception as e:
            rospy.logerr(f"yaml参数文件读取失败! {e}")

        # PID 控制器参数
        self.find_target_velocity = config['find_target_velocity']
        """@const:yaml:自转寻找目标最大速度"""
        pid_param = config['tomid_pid_param']
        """@const:yaml:PID参数list[kp,ki,kd]"""
        self.kp_tomid = pid_param[0] 
        """@const:yaml:kp参数"""
        self.ki_tomid = pid_param[1]  
        """@const:yaml:ki参数"""
        self.kd_tomid = pid_param[2]
        """@const:yaml:kd参数"""  
        # 时间校验的参数
        self.timeout_duration_tomid = config['timeout_duration_tomid']
        """@const:yaml:目标追踪时间，超过则超时"""
        self.timeout_duration_find_target = config['timeout_duration_find_target']
        """@const:yaml:自转寻找目标时间，超过则超时"""
        self.tomid_fresh_period = config['tomid_fresh_period']
        """@const:yaml:tomid刷新周期"""

        '''
        @Description:ros服务参数
        '''
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel发布底盘控制命令
        # ideal guide param 从参数服务器中获取
        self.ideal_tomid = rospy.get_param('ideal_tomid', 0.001)  # ~[0,1] 理想中心到目标点直线的距离(归一化)
        self.tomid_sample_timer = rospy.Timer(rospy.Duration(self.tomid_fresh_period), self.tomid_sample_timer_callback)
        self.tomid_sample_timer._shutdown = True
        #任务开始时间
        self.start_time = None

        #临时变量
        self.integral = 0.0  # 积分项
        self.previous_error = 0.0  # 上一次的误差

        log_info = (
            f"PID Trace node start!\n"
            f"Ideal tomid: {self.ideal_tomid}\n"
            f"tomid_fresh_peroid:{self.tomid_fresh_period}\n"
            f"p&i&d:{self.kp_tomid}\t,{self.ki_tomid},\t{self.kd_tomid}"
        )
        rospy.loginfo(log_info)

    def check_timeout(self, duration):
        """
        :param duration: 超时时间
        :return: null
        :raise: TimeoutError
        """
        if rospy.Time.now().to_sec() - self.start_time > duration:
            raise TimeoutError(f"Timeout! Duration: {duration}")

    def convert_twist(self, x, y, z):
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
        """
        @Describe: 寻找目标板
        @Returns: bool,如果找到目标返回True,否则返回False
        @raises: ValueError 如果rescue的值不正确
        """
        rescue_status = rospy.get_param('rescue', NONGOAL)

        if rescue_status in (FIND,FIND_BUT_NOT_ALLIGN):  # 找到目标
                return True          
        elif rescue_status in (NONGOAL,GOAL_BUT_NOT_TARGET):  # 没有找到目标  
                self.pub.publish(self.convert_twist(0, 0, self.find_target_velocity))
                return False
        else :
            raise ValueError("错误的rescue码值")

    def tomid_sample_timer_callback(self,event):
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
        except rospy.ROSException:
            rospy.logerr("rescue_x 参数不存在!")                                   #程序出口:参数不存在
            return
        except Exception:
            rospy.logerr("LOST")                                                                    #程序出口:目标从视野内丢失
            return                                                                                        

        # 确保 self.tomid 在合理范围内 (-0.5 到 0.5)
        if not (-0.5 <= tomid <= 0.5):
            rospy.logerr(f"self.tomid 值超出范围: {tomid}")
            return                                                                                                #程序出口:tomid 超出范围

        # 归一化 self.tomid 到 [-1, 1]
        normalized_tomid = tomid / 0.5                                               #tomid合理，使用值进行控制更新

        # 检查是否对齐
        if abs(normalized_tomid) <= self.ideal_tomid:
            self.pub.publish(self.convert_twist(0, 0, 0))#停车
            # 重置积分项和上一次的误差
            self.integral = 0.0
            self.previous_error = 0.0
            if DEBUG_MODE:
                rospy.loginfo("已经对齐")
            return                                                                                                       # 程序出口:已对齐

        #未对齐：                                                                                                   对delta_y进行pid

        # 计算PID 控制速度
        error = normalized_tomid
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        pid_output_velocity_y =  - (self.kp_tomid * error + 
                                  self.ki_tomid * self.integral + self.kd_tomid * derivative)
        
        #针对小于0.1的pid输出，做时间pid补偿
        if(abs(pid_output_velocity_y)<0.1):
            """
            #只在速度小于0.1时,做时间控制补偿增量输出
            #param: pid_output_velocity_y_time
            """
            pid_output_velocity_y_time = (abs(pid_output_velocity_y)/0.1) * self.tomid_fresh_period
            #将发布时间速度置回+-0.1
            pid_output_velocity_y = 0.1* pid_output_velocity_y/abs(pid_output_velocity_y)

        else :#pid速度大于0.1的情况:跑满整个间隔
            pid_output_velocity_y_time = self.tomid_fresh_period
            pass
        
        #DEBUG调试终端显示
        if DEBUG_MODE:
            rospy.loginfo(f"pid_output_velocity_y: {pid_output_velocity_y}, \
            time:{pid_output_velocity_y_time}, \t tomid: {tomid}")

        #创建一段时间通过速度控制增量
        time_stamp = rospy.Time.now().to_sec()
        while(rospy.Time.now().to_sec() - time_stamp <pid_output_velocity_y_time):
            self.pub.publish(self.convert_twist(0,pid_output_velocity_y,0))


if __name__ == '__main__' and DEBUG_MODE:
    try:  
        rospy.init_node('PID_BoardTrace_Controller')
        pid_controller = PID_BoardTrace_Controller()
        
        # 寻找目标
        pid_controller.start_time = rospy.Time.now().to_sec()  # 记录开始时间
        while not pid_controller.find_target():
            if TIMEOUT:
                pid_controller.check_timeout(pid_controller.timeout_duration_find_target)

        # 对齐
        pid_controller.start_time = rospy.Time.now().to_sec()  # 记录开始时间
        pid_controller.tomid_sample_timer._shutdown = False#计时器开始开火
        if TIMEOUT:
            while not pid_controller.check_timeout(pid_controller.timeout_duration_tomid):
                pass
        rospy.spin()

    except TimeoutError:
        rospy.signal_shutdown("超时!")
        sys.exit()
    except KeyboardInterrupt:  
        rospy.signal_shutdown("PID BoardTrace Controller节点终止!")
        sys.exit()
    except Exception as e:
        rospy.signal_shutdown("未知错误:{e}")
        sys.exit()