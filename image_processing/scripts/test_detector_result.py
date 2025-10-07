import cv2  # 检测detector所说的标信息: 像素(416,134) 是否正确

# 替换为你的图像路径
image_path = "/home/suda/drone_ugv_ws/src/image_processing/image_save/color_241_832_1.jpg"

# 读取图像
img = cv2.imread(image_path)
if img is None:
    print("无法读取图像，请检查路径")
    exit()

# 目标像素坐标（从日志中获取：416,134）
target_x = 581
target_y = 393

# 在图像上绘制标记：红色圆点（半径5，线宽2），并标注坐标
cv2.circle(img, (target_x, target_y), 5, (0, 0, 255), 2)  # 红色圆点
cv2.putText(img, f"({target_x},{target_y})", (target_x+10, target_y-10), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)  # 标注文字

# 显示图像（按ESC键关闭窗口）
cv2.imshow("Target Pixel Check", img)
while True:
    key = cv2.waitKey(1)
    if key == 27:  # ESC键
        break
cv2.destroyAllWindows()
