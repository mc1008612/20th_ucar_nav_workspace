import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
import sys
import os
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion

class LidarCloudVelocityPub:
    """
    激光雷达数据处理类，用于找板坐标
    @return: 发布'nav_status'参数判断状态
    """
    def __init__(self, config_file_path=os.path.join(os.path.dirname(__file__), 'lidar_debug_config.yaml'), debug_mode=False):
        """
        初始化参数。
        :param config_file_path: 配置文件路径
        :param debug_mode: 是否启用调试模式，默认为 False\n
        NOTE: nav_status:
        用于发布导航状态的参数
        [ERROR,     LIDAR_FOUND,    LIDAR_FIND,   #本文件中相关状态
        FOUND,       FOUND_BUT_NOT_ALIGN,     ALIGN,      NONGOAL,    GOAL_BUT_NOT_TARGET
        #pid_trace.py中发布状态]
        发布:LIDAR_FOUND，发布'nav_status'为'LIDAR_FOUND'，表示激光雷达已经找到目标点
        ERROR 发布'nav_status'为'ERROR'，表示程序出错
        接受:LIDAR_FIND，接受'nav_status'为'LIDAR_FIND'，表示需要激光雷达寻找目标点

        @return self.target_board_point ->PoseStamped
        """
        self.DEBUG_MODE = debug_mode
        self.config_file_path = config_file_path
        self.lidar_config = self.load_config()
        self.cloud_point = None
        self.angle_min = None
        self.angle_max = None
        self.angular_resolution = None
        # 从配置文件中读取参数
        self.point_reach_max = self.lidar_config['point_reach_max']
        self.point_reach_min = self.lidar_config['point_reach_min']
        self.num_points = self.lidar_config['num_points']
        self.tolerance_distance = self.lidar_config['tolorance_distance']
        self.ideal_distance_with_board = self.lidar_config['ideal_distance_with_board']
        self.max_consecutive_distance = self.lidar_config['max_consecutive_distance']

        # 新增属性来存储 plot_points 方法所需的参数
        self.all_points = None
        self.front_points = None
        self.raw_front_points = None
        self.board_points = None
        self.center_point = None
        self.target_point = None
        self.a = None
        self.b = None
        self.norm_vector = None

        # return
        self.target_board_point: PoseStamped = None

    def load_config(self):
        """
        加载配置文件。
        :return: 配置文件内容
        """
        try:
            with open(self.config_file_path, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            print("配置文件不存在，请检查路径是否正确。")
            if not self.DEBUG_MODE:
                rospy.set_param('nav_status', 'ERROR')
            sys.exit(1)  # 程序出口

    def fit_line_to_points(self, points):
        """
        使用 RANSAC 算法拟合直线，并返回方向向量。
        :param points: 包含 (x, y) 坐标的点列表
        :return: 方向向量 (a, b)，表示直线 y = ax + b
        """
        if len(points) < 2:
            raise ValueError("至少需要两个点来拟合直线。")

        x = np.array([p[0] for p in points]).reshape(-1, 1)
        y = np.array([p[1] for p in points])

        ransac = RANSACRegressor()
        ransac.fit(x, y)

        a = ransac.estimator_.coef_[0]
        b = ransac.estimator_.intercept_

        return a, b

    def get_front_points(self, cloud_point, angle_min, angle_max, angular_resolution, num_points):
        """
        从激光雷达数据中提取正前方的指定数量的点。
        :param cloud_point: 激光雷达的距离数据
        :param angle_min: 扫描的起始角度
        :param angle_max: 扫描的结束角度
        :param angular_resolution: 角度分辨率
        :param num_points: 要提取的点的数量
        :return: 正前方的点列表
        """
        front_angle = (angle_min + angle_max) / 2
        half_range = num_points * angular_resolution / 2
        start_angle = front_angle - half_range
        end_angle = front_angle + half_range
        front_points = []
        for i, distance in enumerate(cloud_point):
            if (distance == float('inf') or
                    distance > self.point_reach_max or
                    distance < self.point_reach_min):
                continue
            angle = angle_min + i * angular_resolution
            if start_angle <= angle <= end_angle:
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                front_points.append((x, y))
                if len(front_points) >= num_points:
                    break

        return front_points

    def normalize_vector(self, vector):
        """
        归一化向量。
        :param vector: 需要归一化的向量
        :return: 归一化后的向量
        """
        norm = np.linalg.norm(vector)
        epsilon = 1e-10
        if norm < epsilon:
            return vector
        return vector / norm

    def get_normalized_normal_vector(self, a,root):
        """
        根据直线的斜率 a 计算归一化法向量。
        :param a: 直线的斜率
        :param root: nparray根部点，用于计算方向向量
        :return: 归一化后的法向量，指向内部（原点方向）
        """
        a = np.float64(a)
        _direction_vector = root
        
        # 法向量应该是 [-a, 1] 或 [a, -1] 中的一个
        # 我们假定为[a,-1]
        _v1 = np.array([a, -1.0],dtype= float)
        # 计算  方向向量  与  根部指向原点的方向向量  的点乘
        # 如果点乘结果大于 0，则方向向量和法向量同向，否则反向
        # 因此，我们取点乘结果小于 0 的那个法向量
        if(np.dot(_direction_vector,_v1) < 0):
            _vector = _v1
        else : _vector = -_v1
        # 归一化
        normalized_vector = self.normalize_vector(_vector)

        return normalized_vector
    def point_to_line_distance(self, a, b, point):
        """
        计算直线外一点到直线的距离。
        :param a: 直线的斜率
        :param b: 直线的截距
        :param point: 直线外的点，格式为 (x0, y0)
        :return: 点到直线的距离
        """
        x0, y0 = point
        distance = abs(a * x0 - y0 + b) / np.sqrt(a**2 + 1)
        return distance

    def plot_points(self):
        """
        绘制激光雷达数据点和拟合直线。
        """
        if len(self.front_points) >= 2:
            all_x, all_y = zip(*self.all_points)
            front_x, front_y = zip(*self.front_points)
            fit_y = [self.a * x + self.b for x in all_x]

            plt.clf()  # 清除当前图形
            plt.scatter(all_x, all_y, label='All Points', color='blue', alpha=0.5)
            plt.scatter(self.raw_front_points[:, 0], self.raw_front_points[:, 1], label='Front Points', color='red')
            plt.scatter(front_x, front_y, label='Filtered Points', color='orange')
            plt.scatter(self.board_points[:, 0], self.board_points[:, 1], label='Board Points', color='#fff107', alpha=0.5)
            plt.scatter(self.center_point[0], self.center_point[1], label='Center Point', color='#abc107')
            plt.scatter(self.target_point[0], self.target_point[1], label='Target Point', color='#39c5bb')
            plt.plot(all_x, fit_y, label=f'Fitted Line: y = {self.a:.2f}x + {self.b:.2f}', color='green')

            plt.legend()
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Laser Scan Points and Fitted Line')
            plt.grid(True)
            plt.xlim(-self.point_reach_max, self.point_reach_max)
            plt.ylim(-self.point_reach_max, self.point_reach_max)

            arrow_start = self.center_point
            arrow_end = (self.center_point + self.norm_vector)
            plt.annotate('', xy=arrow_end, xytext=arrow_start,
                         arrowprops=dict(facecolor='blue', shrink=0.05))
            arrow_start = (0, 0)
            arrow_end = (self.point_reach_max / 3, 0)
            plt.annotate('', xy=arrow_end, xytext=arrow_start,
                         arrowprops=dict(facecolor='red', shrink=0.05))

            plt.show()

    def angle_to_quaternion(self, a) -> Quaternion:
        """
        根据绕 Z 轴旋转的角度 a 计算四元数。
        :param a: 旋转角度（单位：rad）
        :return: 四元数Quaternion ( x, y, z,w)
        """
        # 定义旋转轴（Z 轴）
        RAxis = np.array([0, 0, 1])

        # 计算角度的半角
        theta = (a / 2)

        # 计算四元数的各个分量
        w = np.cos(theta)
        x = RAxis[0] * np.sin(theta)
        y = RAxis[1] * np.sin(theta)
        z = RAxis[2] * np.sin(theta)

        return Quaternion(x, y, z, w)

    def run(self):
        """
        程序入口点。
        """
        if not self.DEBUG_MODE:
            while not rospy.get_param('nav_status', '') == 'LIDAR_FIND':
                pass  # 阻塞等待nav_status为LIDAR_FIND
            try:
                rospy.loginfo("Waiting for scan message...")
                self.cloud_point = rospy.wait_for_message('scan', LaserScan, 5)
            except rospy.ROSException:
                rospy.set_param('nav_status', 'ERROR')  # 程序出口
                rospy.signal_shutdown("Failed to get laser scan message.")
                sys.exit(1)
            else:
                rospy.loginfo("Laser scan message received.")
        else:
            sys.path.append(os.path.dirname(__file__))
            try:
                import lidar_cloud_debug as debug
            except ImportError:
                rospy.set_param('nav_status', 'ERROR')  # 程序出口
                rospy.signal_shutdown("lidar_cloud_debug 模块绝对路径不正确")
                sys.exit(1)
            if self.lidar_config['is_random_pointcloud']:
                self.cloud_point = debug.create_test_scan_msg()
            else:
                self.cloud_point = debug.use_test_scan_msg()

        # 预处理完成
        self.angle_min = self.cloud_point.angle_min
        self.angle_max = self.cloud_point.angle_max
        self.angular_resolution = self.cloud_point.angle_increment

        # 获取激光雷达数据转换为直角坐标系存入all_points中
        self.all_points = []
        for i, distance in enumerate(self.cloud_point.ranges):
            if (distance == float('inf') or
                    distance > self.point_reach_max or
                    distance < self.point_reach_min):
                continue  # 跳过无效点
            angle = self.angle_min + i * self.angular_resolution  # 当前点弧度角
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)  # 极坐标转直角坐标
            self.all_points.append((x, y))

        self.all_points = np.array(self.all_points)  # 列表已固定，转换为np数组便于处理

        # 提取前方点
        self.raw_front_points = self.get_front_points(self.cloud_point.ranges, self.angle_min,
                                                     self.angle_max, self.angular_resolution, self.num_points)  # 直接提取前方点
        self.raw_front_points = np.array(self.raw_front_points)
        # 距离均值滤波
        distances = np.linalg.norm(self.raw_front_points, axis=1)
        mean_distance = np.mean(distances)
        self.front_points = self.raw_front_points[distances <= mean_distance]  # 滤波后前方点

        if len(self.front_points) >= 2:
            self.a, self.b = self.fit_line_to_points(self.front_points)
            print(f"拟合直线的方向向量: a = {self.a}, b = {self.b}")
        else:
            print("提取的点数不足，无法拟合直线。")
            if not self.DEBUG_MODE:
                rospy.set_param('nav_status', 'ERROR')  # 程序出口
            sys.exit(1)


        # 根据拟合直线推算板子
        self.board_points = []
        distances = []

        extern_front_point =self.get_front_points(self.cloud_point.ranges, self.angle_min,
                                                     self.angle_max, self.angular_resolution, self.num_points*3)  # 扩大提取前方点的范围到三倍
        extern_front_point = np.array(extern_front_point)
        for point in extern_front_point:
            distances.append(self.point_to_line_distance(self.a, self.b, point))

        tolerance_distances = np.linspace(self.tolerance_distance, self.tolerance_distance, len(extern_front_point))
        self.board_points = extern_front_point[distances <= tolerance_distances]

        if len(self.board_points) > 1:  # 找到多个板子点时，过滤掉不连续点
            for i in range(1, len(self.board_points)):
                if np.linalg.norm(self.board_points[i] - self.board_points[i - 1]) > self.max_consecutive_distance:
                    self.board_points = self.board_points[:i]
                    break

        if self.board_points.size == 0:
            print("未找到符合要求的点，无法确定板子的中心点。")
            sys.exit(1)
        else:#计算中心点
            self.center_point = np.mean(self.board_points, axis=0)

        # 计算法向量
        self.norm_vector = self.get_normalized_normal_vector(self.a,self.center_point)
        # 转换为四元数，对于小车终点方向，应该与板子的法向量相反，故而取负值
        angle =- np.arccos(self.norm_vector[0]) * self.norm_vector[1] / abs(self.norm_vector[1])
        board_orinetation = self.angle_to_quaternion(angle)
        print(f"角度: {angle / np.pi * 180}")
        print(f"四元数: {board_orinetation}")

        # 计算目标点在base_link下的坐标
        self.target_point = self.center_point + self.norm_vector * self.ideal_distance_with_board

        if not self.DEBUG_MODE:  # write导航点参数for others scripts import this module to use
            print('Target:f{self.target_point}\n')
            self.target_board_point = PoseStamped()
            self.target_board_point.header.frame_id = 'base_link'
            self.target_board_point.pose.position.x = self.target_point[0]
            self.target_board_point.pose.position.y = self.target_point[1]
            # normalize_boradVertical_vector = [cos(angle),sin(angle)]
            # RAxis = [0,0,1]绕着z轴旋转
            # q.w=cos((a/2)*pi/180)
            # q.x=RAix.x*sin((a/2)*pi/180)
            # q.y=RAix.y*sin((a/2)*pi/180)
            # q.z=RAix.z*sin((a/2)*pi/180)
            rospy.set_param('nav_status', 'LIDAR_FOUND')
            self.target_board_point.pose.orientation = board_orinetaion
            # 更新导航参数状态

if __name__ == '__main__':
    config_file_path = os.path.join(os.path.dirname(__file__), 'lidar_debug_config.yaml')
    lidar_processor = LidarCloudVelocityPub(config_file_path, True)
    lidar_processor.run()
    lidar_processor.plot_points()