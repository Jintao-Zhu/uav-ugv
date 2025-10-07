import numpy as np
from PIL import Image
import yaml

# -----------------------------
# 参数配置
# -----------------------------
yaml_path = "/home/suda/drone_ugv_ws/src/yahboomcar_nav/maps/yahboomcar.yaml"
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml['resolution']         # 米/像素
origin_x, origin_y, origin_yaw = map_yaml['origin']
image_path = "/home/suda/drone_ugv_ws/src/yahboomcar_nav/maps/yahboomcar.pgm"

wall_height = 1.0        # 墙体高度
min_segment_length = 3   # 最小合并像素数（超过此值才合并，太小的保留为单个）
output_sdf_path = "/home/suda/drone_ugv_ws/src/map_struct/models/auto_walls/model.sdf"

# -----------------------------
# 读取地图并提取黑色像素
# -----------------------------
image = Image.open(image_path).convert('L')
map_data = np.array(image)
height, width = map_data.shape

# 提取所有黑色像素坐标 (y, x)
black_pixels = np.where(map_data < 30)  # 只保留纯黑色
pixel_count = len(black_pixels[0])
print(f"检测到 {pixel_count} 个黑色像素")

# 将像素坐标转换为列表并排序
pixels = list(zip(black_pixels[1], black_pixels[0]))  # (x, y) 格式
pixels.sort()  # 按x坐标排序，便于横向合并

# -----------------------------
# 合并相邻像素为连续线段/矩形
# -----------------------------
merged_segments = []
current_segment = []

for px, py in pixels:
    if not current_segment:
        current_segment.append((px, py))
    else:
        # 检查是否相邻（右邻或下邻）
        last_px, last_py = current_segment[-1]
        if (px == last_px + 1 and py == last_py) or (py == last_py + 1 and px == last_px):
            current_segment.append((px, py))
        else:
            merged_segments.append(current_segment)
            current_segment = [(px, py)]
if current_segment:
    merged_segments.append(current_segment)

# -----------------------------
# 生成优化后的墙体SDF
# -----------------------------
sdf_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="optimized_walls">
'''

segment_id = 0
for seg in merged_segments:
    # 提取线段的边界框
    xs = [p[0] for p in seg]
    ys = [p[1] for p in seg]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    seg_length = len(seg)
    
    # 计算实际坐标和尺寸
    center_x = origin_x + (min_x + max_x) / 2 * resolution
    center_y = origin_y + (height - (min_y + max_y) / 2) * resolution  # 保持正确坐标映射
    size_x = (max_x - min_x + 1) * resolution
    size_y = (max_y - min_y + 1) * resolution
    
    # 小线段用多个小方块，长线段用一个大矩形
    if seg_length <= min_segment_length:
        # 短线段：保留单个像素精度
        for i, (px, py) in enumerate(seg):
            world_x = origin_x + px * resolution
            world_y = origin_y + (height - py) * resolution
            sdf_content += f'''
    <link name="wall_small_{segment_id}_{i}">
      <pose>{world_x:.4f} {world_y:.4f} {wall_height/2:.4f} 0 0 0</pose>
      <collision name="collision">
        <geometry><box><size>{resolution} {resolution} {wall_height}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{resolution} {resolution} {wall_height}</size></box></geometry>
        <material><script><name>Gazebo/Grey</name></script></material>
      </visual>
    </link>
            '''
    else:
        # 长线段：合并为一个矩形
        sdf_content += f'''
    <link name="wall_large_{segment_id}">
      <pose>{center_x:.4f} {center_y:.4f} {wall_height/2:.4f} 0 0 0</pose>
      <collision name="collision">
        <geometry><box><size>{size_x:.4f} {size_y:.4f} {wall_height}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size_x:.4f} {size_y:.4f} {wall_height}</size></box></geometry>
        <material><script><name>Gazebo/Grey</name></script></material>
      </visual>
    </link>
        '''
    segment_id += 1

# 闭合SDF文件
sdf_content += '''
  </model>
</sdf>
'''

# -----------------------------
# 保存文件
# -----------------------------
with open(output_sdf_path, 'w') as f:
    f.write(sdf_content)

print(f"✅ 优化后生成 {segment_id} 个墙体单元（原始像素 {pixel_count} 个）")
print(f"📦 生成文件路径：{output_sdf_path}")
