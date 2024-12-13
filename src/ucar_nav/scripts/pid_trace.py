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
FIND = 4

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
        except Exception as e:
            rospy.logerr(f"yaml参数文件读取失败! {e}")

        # PID velocity param
        self.align_tomid_velocity = config['align_tomid_velocity']
        self.find_target_velocity = config['find_target_velocity']
        pid_param = config['tomid_pid_param']
        self.kp_tomid = pid_param[0]  # 添加 kp 参数
        self.ki_tomid = pid_param[1]  # 添加 ki 参数
        self.kd_tomid = pid_param[2]  # 添加 kd 参数
        # time out check param
        self.timeout_duration_tomid = config['timeout_duration_tomid']
        self.timeout_duration_find_target = config['timeout_duration_find_target']
        #tomid刷新率
        self.tomid_change_rate = config['tomid_change_rate']

        '''
        ros服务参数
        '''
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel发布底盘控制命令
        # ideal guide param 从参数服务器中获取
        self.ideal_tomid = rospy.get_param('ideal_tomid', 0.001)  # ~[0,1] 理想中心到目标点直线的距离(归一化)
        self.pub_timer_tomid = rospy.Timer(rospy.Duration(0.1), self.pub_tomid_velocity) #10Hz发布速率
        #开启定时器，然后关闭回调
        self.pub_timer_tomid.start()
        self.pub_timer_tomid.shutdown()
        #任务开始时间
        self.start_time = None

        #临时变量
        self.integral = 0.0  # 积分项
        self.previous_error = 0.0  # 上一次的误差
        self.tomid = 0.0 #记录tomid
        self.pid_output_velocity_y = 0.0 # 用于记录发布的当前y轴方向上的速度
        self.pid_output_velocity_y_time = 0.0 #用于记录应该发布速度的持续时间
        self.tomid_pub_time_stamp = None

        log_info = (
            f"PID Trace node start!\n"
            f"Ideal tomid: {self.ideal_tomid}\n"
            f"Align to mid velocity: {self.align_tomid_velocity}"
        )
        rospy.loginfo(log_info)

    def check_timeout(self, duration):
        """
        :param duration: 超时时间
        :return: null
        :raise: TimeoutException
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
        self.pub.publish(self.convert_twist(0,self.pid_output_velocity_y,0))

    def change_tomid_velocity(self):
        # 获取当前 self.tomid 值
        try:
            now_tomid = rospy.get_param('rescue_x',0)
            if(self.tomid == now_tomid #检查tomid更新
               and now_tomid != 0):#排除初值&对齐情况
                raise Exception
        except rospy.ROSException:
            rospy.logerr("rescue_x 参数不存在!")
            return False                                                                                        #程序出口:参数不存在
        except Exception:
            rospy.logerr("rescue_x 参数没有更新!")
            #rescue_x参数不更新，不执行
            return False                                                                                         #程序出口:参数不更新

            
        # tomid变更时处理:
        self.tomid = now_tomid
        # 确保 self.tomid 在合理范围内 (-0.5 到 0.5)
        if not (-0.5 <= self.tomid <= 0.5):
            rospy.logerr(f"self.tomid 值超出范围: {self.tomid}")
            return False                                                                                          #程序出口:tomid 超出范围

        # 归一化 self.tomid 到 [-1, 1]
        normalized_tomid = self.tomid / 0.5                                             #tomid合理，使用值进行控制更新

        # 检查是否对齐
        if abs(normalized_tomid) <= self.ideal_tomid:
            self.pub.publish(self.convert_twist(0, 0, 0))
            # 重置积分项和上一次的误差
            self.integral = 0.0
            self.previous_error = 0.0
            if DEBUG_MODE:
                rospy.loginfo("已经对齐")
            return True                                                                                           # 程序出口:已对齐

        #未对齐：                                                                                                   进行状态更改

        #先关闭发送定时器
        self.pub_timer_tomid._shutdown = True

        # 计算PID 控制速度
        error = normalized_tomid
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        self.pid_output_velocity_y = -   (self.kp_tomid * error + 
                                  self.ki_tomid * self.integral + self.kd_tomid * derivative)
        
        #针对小于0.1的pid输出，做时间pid补偿
        if(self.pid_output_velocity_y<0.1):
            """
            #只在速度小于0.1时,做pid补偿输出,使用时间pid对象
            #param: pid_output_velocity_y_time
            """
            self.pid_output_velocity_y_time = 0.1*(1/self.tomid_change_rate) / self.pid_output_velocity_y
            #将发布时间速度置回0.1
            self.pid_output_velocity_y = 0.1

            #使用定时器发布
            #如果时间戳为空，创建时间戳
            if self.tomid_pub_time_stamp == None:
                self.tomid_pub_time_stamp = rospy.Time.now().to_sec()
            self.pub_timer_tomid._shutdown = False  #开启定时器发送速度指令
            if(not #如果超过设定的发送时间，则停止定时器
            rospy.Time.now().to_sec() - self.tomid_pub_time_stamp > self.pid_output_velocity_y_time):
                self.pub.publish(self.convert_twist(0,0,0))#停车
                self.pub_timer_tomid._shutdown = True#关闭定时器回调发送速度指令
                self.tomid_pub_time_stamp = None#清空时间戳

        else :#pid速度大于0.1的情况
            #开启定时器回调函数一直发送
            self.pub_timer_tomid._shutdown = False

        #DEBUG调试终端显示
        if DEBUG_MODE:
            rospy.loginfo(f"self.pid_output_velocity_y: {self.pid_output_velocity_y}, self.tomid: {self.tomid}")
        return False


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
        while not pid_controller.change_tomid_velocity():
            if TIMEOUT:
                pid_controller.check_timeout(pid_controller.timeout_duration_tomid)
        
    except TimeoutError:
        rospy.signal_shutdown("超时!")
        exit(1)
    except KeyboardInterrupt:  
        rospy.signal_shutdown("PID BoardTrace Controller node terminated!")
        exit(2)