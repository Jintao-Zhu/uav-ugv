import cv2
import numpy as np

# 创建一张480x848的黑色图像（高度480，宽度848）
img = np.zeros((480, 848, 3), dtype=np.uint8)  # 注意：OpenCV中维度是 (高度, 宽度, 通道)

# 在Y=0（顶部中间）画红色圆点
cv2.circle(img, (424, 0), 5, (0, 0, 255), -1)  # (x=424, y=0)

# 在Y=479（底部中间）画绿色圆点
cv2.circle(img, (424, 479), 5, (0, 255, 0), -1)  # (x=424, y=479)

# 保存并显示图像
cv2.imwrite("test_coords.jpg", img)
cv2.imshow("Coordinate Test", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
