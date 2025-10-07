import cv2
import numpy as np

# 读取XML格式的32FC1深度图
depth_path = "/home/suda/drone_ugv_ws/src/image_processing/image_save/depth_237_731_0.xml"
fs = cv2.FileStorage(depth_path, cv2.FILE_STORAGE_READ)  # 注意这里是FILE_STORAGE_READ，不是C++的::READ

if not fs.isOpened():
    print("无法读取深度图XML文件")
else:
    depth_img = fs.getNode("depth_image").mat()  # 获取原始32FC1矩阵
    fs.release()  # 关闭文件存储
    
    print("数据类型:", depth_img.dtype)  # 应输出float32
    print("图像尺寸:", depth_img.shape)  # 应输出(480, 848)
    print("中心像素值:", round(depth_img[240, 424], 3))  # 打印中心位置的深度值
    print("像素值范围:", round(np.min(depth_img), 3), "~", round(np.max(depth_img), 3))
