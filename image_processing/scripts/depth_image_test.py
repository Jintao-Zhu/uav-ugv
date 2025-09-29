import cv2
import numpy as np

depth_img = cv2.imread("/home/suda/drone_ugv_ws/src/image_processing/image_save/depth_1221_436_0.png", cv2.IMREAD_UNCHANGED)

# 过滤无效值（通常深度图中“0”代表无效距离）
valid_depth = depth_img[depth_img > 0]
if len(valid_depth) == 0:
    print("无有效深度数据")
else:
    # 归一化：将有效距离范围映射到0~255
    min_dist = valid_depth.min()
    max_dist = valid_depth.max()
    depth_normalized = ((depth_img - min_dist) / (max_dist - min_dist) * 255).astype(np.uint8)
    
    # 显示归一化后的深度图（此时在VS Code的图像窗口中可看到明暗梯度）
    cv2.imshow("Normalized Depth Map", depth_normalized)
    cv2.waitKey(0)
