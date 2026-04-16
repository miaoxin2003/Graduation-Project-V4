import cv2
import csv
import time
from HandTrackingModule import handDetector

# 初始化
cap = cv2.VideoCapture(1)
detector = handDetector()
label = input("请输入当前手势标签 (例如: OK, Like, Peace): ")
file_name = "dataset.csv"

print(f"正在采集手势: {label}")
print("按 's' 保存当前坐标, 按 'q' 退出")

with open(file_name, mode='a', newline='') as file:
    writer = csv.writer(file)
    
    while True:
        success, img = cap.read()
        if not success:
            break
            
        img = detector.findHands(img)
        lmList = detector.findPosition(img, draw=False)
        
        cv2.imshow("Image", img)
        key = cv2.waitKey(1) & 0xFF  # 添加掩码确保跨平台兼容
        
        if key == ord('s'):
            if len(lmList) != 0:
                # 提取 21 个点的 x, y 坐标（归一化到[0,1]）
                h, w = img.shape[:2]
                data = [label]
                for lm in lmList:
                    norm_x = lm[1] / w  # 归一化x
                    norm_y = lm[2] / h  # 归一化y
                    data.extend([norm_x, norm_y])
                writer.writerow(data)
                print(f"已保存: {label} (归一化坐标)")
            else:
                print("未检测到手部")
                
        elif key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()