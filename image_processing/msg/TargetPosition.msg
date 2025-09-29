# TargetPosition.msg
std_msgs/Header header          # 时间戳和坐标系
int32 class_id                  # 目标类别ID
float64 confidence              # 检测置信度
float64 x_world                 # 世界坐标系X轴（单位：米）
float64 y_world                 # 世界坐标系Y轴（单位：米）
float64 z_world                 # 世界坐标系Z轴（单位：米）
float64 gps_lat                 # 目标对应GPS纬度
float64 gps_lon                 # 目标对应GPS经度