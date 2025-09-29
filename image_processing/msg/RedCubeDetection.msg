# 红色立方体检测结果消息
std_msgs/Header header          # 消息头（包含时间戳、坐标系）

# 检测目标基本信息
int32 class_id                  # 目标类别ID
string class_name               # 目标类别名称
float32 confidence              # 检测置信度

# 边界框坐标（像素坐标系）
int32 xmin                      # 左上角x
int32 ymin                      # 左上角y
int32 xmax                      # 右下角x
int32 ymax                      # 右下角y
int32 center_x                  # 目标中心点x
int32 center_y                  # 目标中心点y

# 目标在相机坐标系下的3D坐标(单位:米)
float32 cam_x
float32 cam_y
float32 cam_z

# 新增：目标在世界坐标系下的3D坐标(单位:米)
# （由TF2从相机坐标系转换而来，转换失败时为NaN）
float32 world_x
float32 world_y
float32 world_z

# 无人机位姿信息（来自采集节点）
float32 uav_x                   # 无人机位置x
float32 uav_y                   # 无人机位置y
float32 uav_z                   # 无人机位置z
float32 uav_qx                  # 无人机姿态四元数x
float32 uav_qy                  # 无人机姿态四元数y
float32 uav_qz                  # 无人机姿态四元数z
float32 uav_qw                  # 无人机姿态四元数w