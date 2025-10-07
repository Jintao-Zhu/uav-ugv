import cv2
import sys

# 替换为你保存的彩色图像路径（从color_images目录选一张）
image_path = "/home/suda/drone_ugv_ws/src/image_processing/image_save/depth_241_332_0.png"
# 读取图像并获取分辨率
img = cv2.imread(image_path)
if img is None:
    print(f"错误：无法读取图像文件 {image_path}")
    sys.exit(1)

# 获取宽（cols）和高（rows）
height, width = img.shape[:2]
print(f"图像分辨率：宽={width}，高={height}")
print(f"是否为640×480：{'是' if width==640 and height==480 else '否'}")
