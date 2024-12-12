from sensor_msgs.msg import LaserScan
import numpy as np
import yaml


def create_test_scan_msg():
    # 创建一个 LaserScan 消息实例
    scan_msg = LaserScan()
    
    # 设置激光雷达的基本参数
    with open('src/ucar_nav/scripts/lidar_debug_config.yaml', 'r') as file:
        lidar_config = yaml.safe_load(file)

    scan_msg.header.frame_id = lidar_config['header.frame_id']
    scan_msg.angle_min = lidar_config['angle_min']
    scan_msg.angle_max = lidar_config['angle_max']
    scan_msg.angle_increment = lidar_config['angle_increment']
    scan_msg.time_increment = lidar_config['time_increment']
    scan_msg.scan_time = lidar_config['scan_time']
    scan_msg.range_min = lidar_config['range_min']
    scan_msg.range_max = lidar_config['range_max']
    
    # 生成一些测试数据，包含 inf 值
    ranges = np.random.uniform(scan_msg.range_max, scan_msg.range_max, int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment))
    # 在某些位置插入 inf 值
    ranges[::10] = float('inf')  # 每隔 10 个点插入一个 inf 值
    
    scan_msg.ranges = ranges.tolist()
    scan_msg.intensities = []  # 如果需要，可以添加强度数据
    
    return scan_msg


def use_test_scan_msg(rotation_angle=-np.pi/10):
    # 参数读取
    with open('src/ucar_nav/scripts/lidar_debug_config.yaml', 'r') as file:
        lidar_config = yaml.safe_load(file)

    # 创建一个 LaserScan 消息实例
    scan_msg = LaserScan()
    
    # 设置激光雷达的基本参数
    scan_msg.header.frame_id = lidar_config['header.frame_id']
    scan_msg.angle_min = lidar_config['angle_min']
    scan_msg.angle_max = lidar_config['angle_max']
    scan_msg.angle_increment = lidar_config['angle_increment']
    scan_msg.time_increment = lidar_config['time_increment']
    scan_msg.scan_time = lidar_config['scan_time']
    scan_msg.range_min = lidar_config['range_min']
    scan_msg.range_max = lidar_config['range_max']
    
    fixed_ranges = [
    float('inf'), float('inf'), 2.470750093460083, 2.4709999561309814, 2.4744999408721924, 
    2.4790000915527344, 2.489000082015991, 2.499000072479248, 2.5072500705718994, 2.515000104904175, 
    2.53125, 2.5490000247955322, 2.5490000247955322, float('inf'), float('inf'), float('inf'), 
    float('inf'), float('inf'), float('inf'), 4.008500099182129, 4.007999897003174, 4.007750034332275, 
    0.0, float('inf'), float('inf'), float('inf'), 0.0, 4.240250110626221, 4.241000175476074, 
    4.238500118255615, 4.235000133514404, 4.234499931335449, float('inf'), 3.6837499141693115, 
    3.684000015258789, 3.66825008392334, 3.6530001163482666, 3.5485000610351562, 3.443000078201294, 
    3.442500114440918, 0.0, 0.0, 0.0, 3.4942500591278076, 3.49399995803833, 3.4932498931884766, 
    float('inf'), float('inf'), 0.0, 0.0, 0.0, 3.4554998874664307, 3.4560000896453857, 3.443000078201294, 
    3.430999994277954, 3.5022499561309814, 3.5739998817443848, float('inf'), 0.0, float('inf'), 
    float('inf'), float('inf'), 0.0, 2.0537500381469727, 2.053999900817871, 2.0537500381469727, 
    float('inf'), float('inf'), float('inf'), float('inf'), 3.4005000591278076, 3.4010000228881836, 
    3.4040000438690186, 3.4079999923706055, 3.413749933242798, 3.4189999103546143, 3.41825008392334, 
    float('inf'), 3.184999942779541, 3.184999942779541, 3.184499979019165, float('inf'), 0.0, 0.0, 
    2.9482500553131104, 2.9489998817443848, 2.9317500591278076, 2.9159998893737793, 2.8980000019073486, 
    2.880000114440918, 2.866499900817871, 2.8519999980926514, 2.83774995803833, 2.8239998817443848, 
    2.8202500343322754, 2.815999984741211, 2.810499906539917, 2.805000066757202, 2.7980000972747803, 
    2.7920000553131104, 2.791749954223633, float('inf'), 0.0, 0.0, 3.2982499599456787, 3.2990000247955322, 
    3.2985000610351562, 0.0, 3.618499994277954, 3.618000030517578, 3.665250062942505, 3.7119998931884766, 
    3.713749885559082, 3.7170000076293945, 3.7165000438690186, 3.7170000076293945, 3.7174999713897705, 
    float('inf'), 3.723249912261963, 3]

    # 计算需要在头尾添加的 inf 数量
    total_length = 360
    fixed_length = len(fixed_ranges)
    padding_length = (total_length - fixed_length) // 2

    # 构造新的 ranges 列表
    ranges = [float('inf')] * padding_length + fixed_ranges + [float('inf')] * padding_length

    # 如果总长度不为偶数，额外添加一个 inf 到末尾
    if len(ranges) < total_length:
        ranges.append(float('inf'))

    # 旋转 ranges 数组
    num_points = len(ranges)
    shift = int((rotation_angle / scan_msg.angle_increment) % num_points)
    ranges = ranges[-shift:] + ranges[:-shift]

    scan_msg.ranges = ranges

    return scan_msg