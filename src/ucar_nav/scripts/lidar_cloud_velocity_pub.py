import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
import sys
import yaml

# sys.argv空参处理
sys.argv.append('debug')
sys.argv.append('plot')                     #默认为debug模式，并绘制点云图
DEBUG_MODE  =  sys.argv[1] == 'debug'
PLOT = sys.argv[2] == 'plot'
print(sys.argv,'\ndebug mode:',str(DEBUG_MODE),'\n\
            plot:',str(PLOT))


# 参数读取
try:
    with open('src/ucar_nav/scripts/lidar_debug_config.yaml', 'r') as file:
        lidar_config = yaml.safe_load(file)
except FileNotFoundError:
    print("配置文件不存在，请检查路径是否正确。")

def fit_line_to_points(points):
    """
    使用 RANSAC 算法拟合直线，并返回方向向量。
    :param points: 包含 (x, y) 坐标的点列表
    :return: 方向向量 (a, b)，表示直线 y = ax + b
    """
    if len(points) < 2:
        raise ValueError("至少需要两个点来拟合直线。")

    # 提取 x 和 y 坐标
    x = np.array([p[0] for p in points]).reshape(-1, 1)
    y = np.array([p[1] for p in points])

    # 使用 RANSAC 进行鲁棒拟合
    ransac = RANSACRegressor()
    ransac.fit(x, y)

    # 获取拟合直线的参数
    a = ransac.estimator_.coef_[0]
    b = ransac.estimator_.intercept_

    return a, b

def get_front_points(cloud_point, angle_min, angle_max, angular_resolution, num_points):
    """
    从激光雷达数据中提取正前方的指定数量的点。
    :param cloud_point: 激光雷达的距离数据
    :param angle_min: 扫描的起始角度
    :param angle_max: 扫描的结束角度
    :param angular_resolution: 角度分辨率
    :param num_points: 要提取的点的数量
    :return: 正前方的点列表
    """
    # 计算正前方的角度范围
    front_angle = (angle_min + angle_max) / 2
    half_range = num_points * angular_resolution / 2
    start_angle = front_angle - half_range
    end_angle = front_angle + half_range

    front_points = []
    for i, distance in enumerate(cloud_point):
        if (distance ==  float('inf') or
            distance > point_reach_max or
            distance < point_reach_min):
            continue
        angle = angle_min + i * angular_resolution
        if start_angle <= angle <= end_angle:
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            front_points.append((x, y))
            if len(front_points) >= num_points:
                break

    return front_points

def normalize_vector(vector):
    """
    归一化向量。
    :param vector: 需要归一化的向量
    :return: 归一化后的向量
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        #对零向量不进行归一化处理
        return vector
    return vector / norm

def get_normalized_normal_vector(a):
    """
    根据直线的斜率 a 计算归一化法向量。
    :param a: 直线的斜率
    :return: 归一化后的法向量
    """
    # 直线方向向量为(1,a)
    # 朝内法向量为 (a, -1)
    normal_vector = np.array([a, -1])
    # 归一化法向量
    normalized_normal_vector = normalize_vector(normal_vector)
    return normalized_normal_vector

def point_to_line_distance(a, b, point):
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

def plot_points(all_points, front_points, raw_front_points, board_points, center_point, target_point, a, b, point_reach_max):
    """
    绘制激光雷达数据点和拟合直线。
    :param all_points: 所有激光雷达数据点
    :param front_points: 正前方的激光雷达数据点
    :param raw_front_points: 未滤波前的正前方的激光雷达数据点
    :param board_points: 板子的激光雷达数据点
    :param center_point: 板子的中心点
    :param target_point: 目标点的坐标
    :param a: 直线的斜率
    :param b: 直线的截距
    """
    if len(front_points) >= 2:
        # 提取所有点的坐标
        all_x, all_y = zip(*all_points)

        # 提取正前方点的坐标
        front_x, front_y = zip(*front_points)

        # 计算拟合直线的 y 值
        fit_y = [a * x + b for x in all_x]

        # 绘制所有点
        plt.scatter(all_x, all_y, label='All Points', color='blue', alpha=0.5)
        # 绘制前方点
        plt.scatter(raw_front_points[:,0], raw_front_points[:,1], label='Front Points', color='red')
        # 绘制滤波后的点
        plt.scatter(front_x, front_y, label='Filtered Points', color='orange')
        # 绘制板子点
        plt.scatter(board_points[:,0], board_points[:,1], label='Board Points', color='#fff107',alpha=0.5)
        # 绘制板子中心点
        plt.scatter(center_point[0], center_point[1], label='Center Point', color='#abc107')
        # 绘制目标点
        plt.scatter(target_point[0], target_point[1], label='Target Point', color='#39c5bb')
        # 绘制拟合直线
        plt.plot(all_x, fit_y, label=f'Fitted Line: y = {a:.2f}x + {b:.2f}', color='green')

        # 添加图例和标签
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Laser Scan Points and Fitted Line')
        plt.grid(True)

        # 设置 X 和 Y 轴的范围
        plt.xlim(-point_reach_max, point_reach_max)
        plt.ylim(-point_reach_max, point_reach_max)
        
        # 在 x 轴上绘制红色箭头表示扫描方向
        arrow_start = (0, 0)
        arrow_end = (point_reach_max/3, 0)
        plt.annotate('', xy=arrow_end, xytext=arrow_start,
                     arrowprops=dict(facecolor='red', shrink=0.05))
        
        # 显示图形
        plt.show()
        rospy.signal_shutdown("Program terminated.")

if  __name__ == '__main__':
    while not rospy.is_shutdown():
                if not DEBUG_MODE:#run mode
                    rospy.init_node('scan_subscriber')
                    # # get cloud_point from laser scan
                    try:
                        rospy.loginfo("Waiting for scan message...")
                        cloud_point = rospy.wait_for_message('scan', LaserScan,5)           #/ 订阅激光雷达数据
                    except rospy.ROSException:
                        rospy.signal_shutdown("Failed to get laser scan message.")
                    else:
                        rospy.loginfo("Laser scan message received.")
                                                                                                                                                            #OR
                else:#debug mode
                        print("debug mode")
                        sys.path.append('/home/minecraft1008612/Desktop/ucar_ws/src/ucar_nav/scripts')
                        try:
                            import lidar_cloud_debug as debug
                        except ImportError:
                            print("检查lidar_cloud_debug 模块绝对路径是否正确")
                        print("execute debug mode")
                        if lidar_config['is_random_pointcloud']:
                            cloud_point = debug.create_test_scan_msg()                                       #/  随机生成测试数据
                        else:
                            cloud_point = debug.use_test_scan_msg()                                             
                
                # 激光雷达参数
                angle_min = cloud_point.angle_min  # 扫描起始角度
                angle_max = cloud_point.angle_max  # 扫描结束角度
                angular_resolution = cloud_point.angle_increment  # 角度分辨率
                #点云滤波参数
                point_reach_max = lidar_config['point_reach_max']
                point_reach_min = lidar_config['point_reach_min']
                num_points = lidar_config['num_points']
                tolorance_distance = lidar_config['tolorance_distance']
                #航点参数
                ideal_distance_with_board = lidar_config['ideal_distance_with_board']

                # 提取所有点
                all_points = []
                for i, distance in enumerate(cloud_point.ranges):
                    if (distance ==  float('inf') or
                                    distance > point_reach_max or
                                    distance < point_reach_min):
                        continue
                    angle = angle_min + i * angular_resolution
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    all_points.append((x, y))

                # 提取正前方的10个点
                raw_front_points = get_front_points(cloud_point.ranges, angle_min, \
                                                    angle_max, angular_resolution, num_points)

                #对前方点进行滤波
                # 计算每个点到原点的距离
                distances = []
                try:
                    distances = np.linalg.norm(raw_front_points, axis=1)
                except:
                    print("raw_front_points为空")
                    sys.exit(1)
                # 计算距离的均值
                mean_distance = np.mean(distances)
                # 过滤掉距离大于均值的点
                raw_front_points = np.array(raw_front_points)
                front_points = raw_front_points[distances <= mean_distance]

                # 拟合直线
                if len(front_points) >= 2:
                    a, b = fit_line_to_points(front_points)
                    print(f"拟合直线的方向向量: a = {a}, b = {b}")
                else:
                    print("提取的点数不足，无法拟合直线。")
                    sys.exit(1)
                #拟合结果存储在：a,b
                #取法向量：
                normalize_boradVertical_vector = get_normalized_normal_vector(a)

                # 根据拟合直线猜测板子所有点位置
                board_points = []
                distances = []

                for point in all_points:
                    distances.append(point_to_line_distance(a, b, point))

                all_points = np.array(all_points)
                tolerance_distances = np.linspace(tolorance_distance, tolorance_distance, len(all_points))
                board_points = all_points[distances <= tolerance_distances]

                # 检查board_points中每对连续点之间的距离
                max_consecutive_distance = lidar_config['max_consecutive_distance']  #板子点最大连续距离，一般等于激光雷达扫描分辨率*10

                if len(board_points) > 1:
                    for i in range(1, len(board_points)):
                        if np.linalg.norm(board_points[i] - board_points[i - 1]) > max_consecutive_distance:
                            board_points = board_points[:i]
                            break
                
                #确定板子中心点位置
                if board_points.size == 0:
                    print("未找到符合要求的点，无法确定板子的中心点。")
                    sys.exit(1)
                    break
                else:
                    center_point =  np.mean(board_points, axis=0)
                #计算base_link坐标系下目标点位置
                target_point = center_point + normalize_boradVertical_vector*ideal_distance_with_board

                if PLOT:
                    plot_points(all_points, front_points,raw_front_points,\
                                board_points,center_point, target_point,a,b, \
                                point_reach_max)

                if not DEBUG_MODE:
                    rospy.set_param('target_board_point',target_point)
                
                rospy.signal_shutdown("Program terminated.")

