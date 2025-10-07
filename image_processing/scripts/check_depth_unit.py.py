import cv2 #查看深度图的单位
import numpy as np

# 深度图路径（
depth_image_path = "/home/suda/drone_ugv_ws/src/image_processing/image_save/depth_252_432_5.png"

# 读取深度图（注意：深度图通常是单通道16位或32位图像）
depth_img = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)

if depth_img is None:
    print(f"错误：无法读取深度图 {depth_image_path}")
    exit(1)

# 目标像素坐标（来自日志：像素(445,364)）
target_x, target_y = 445, 364

# 检查坐标是否在图像范围内
if (target_x < 0 or target_x >= depth_img.shape[1] or
    target_y < 0 or target_y >= depth_img.shape[0]):
    print(f"错误：像素坐标({target_x},{target_y})超出图像范围")
    exit(1)

# 获取原始深度值
raw_depth = depth_img[target_y, target_x]

# 打印结果
print(f"深度图尺寸：{depth_img.shape}（高x宽）")
print(f"像素({target_x},{target_y})的原始深度值：{raw_depth}")
print(f"当前代码解析的深度（按1000缩放）：{raw_depth / 1000.0} m")
print(f"按100缩放（厘米单位）的深度：{raw_depth / 100.0} m")
print(f"按1缩放（毫米单位）的深度：{raw_depth / 1.0} mm")
