import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.filterwarnings("ignore")
import cv2
import mediapipe as mp
import time
import serial
import joblib
import sys
sys.path.append('.')
from FuzzyPID import FuzzyPID, e_grid, de_grid  # 3级FPID
import numpy as np


class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.x_history = []
        self.y_history = []
        self.window_size = 5

        # ========== 自适应EMA滤波参数 ==========
        # EMA基础系数：运动慢时增大（增强平滑），运动快时减小（减少延迟）
        self.alpha_fast = 0.15   # 快速运动时的alpha
        self.alpha_slow = 0.05    # 慢速运动时的alpha
        self.velocity_threshold = 15  # 速度阈值，像素/帧

        # EMA滤波状态
        self.ema_x = None
        self.ema_y = None

        # 历史速度用于计算加速度
        self.velocity_x_history = []
        self.velocity_y_history = []
        self.max_velocity_history = 5

        # ========== 预测控制参数 ==========
        self.pred_x = None        # 预测位置
        self.pred_y = None
        self.last_ema_x = None     # 上一帧EMA值（用于计算速度）
        self.last_ema_y = None
        # ========== FuzzyPID 参数调整补偿 ==========
        # 误差历史（用于计算误差变化率de）
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_time = None
        # FuzzyPID实例（已初始化）
        self.fpid_x = FuzzyPID()
        self.fpid_y = FuzzyPID()
        # 控制量补偿开关
        self.use_fuzzy_compensation = True
        # ======================================

        self.prev_smooth_x = None
        self.prev_smooth_y = None
        self.frame_cnt = 0
        self.frame_count = 0       # 帧计数器
        self.use_prediction = True  # 是否使用预测
        # 预测速度阈值：速度低于此值时不预测（防止静止时噪声）
        self.min_velocity_for_prediction = 2
        # 预测误差容忍度：预测值与平滑值的偏差超过此值时不信任预测
        self.prediction_tolerance = 80
        # ======================================


        # ========== 手势状态机参数 ==========
        # 状态：WAITING(等待) -> CONFIRMING(确认中) -> TRIGGERED(已触发) -> WAITING
        self.gesture_state = "WAITING"
        self.last_gesture = None      # 上一次检测到的手势
        self.confirmed_gesture = None # 已确认的手势（触发后）
        self.hold_start_time = None   # 开始保持的时间戳
        self.hold_duration = 0.5      # 触发需要的保持时间（秒）
        self.cooldown_start = None    # 冷却开始时间
        self.cooldown_duration = 1.0 # 冷却时间（秒），防止重复触发
        # ======================================

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode,
                                        max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):

        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                # print(id, cx, cy)
                lmList.append([id, cx, cy])
                if draw:
                    if id == 4:  # Thumb tip
                        cv2.circle(img, (cx, cy), 15, (0, 0, 139), cv2.FILLED) # Dark Blue
                    else:
                        cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED) # Green

        return lmList


def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(1)
    detector = handDetector()

    # Serial port initialization
    ser = None # Initialize ser to None
    try:
        ser = serial.Serial()
        ser.baudrate = 115200
        ser.port = 'COM8'
        ser.open()
        # Send initial center coordinate to keep pan-tilt stable
        initial_data = '#'+str('320')+'$'+str('240')+'\r\n'
        ser.write(initial_data.encode())
        #print("Serial port opened successfully and initial data sent.")
    except serial.SerialException as e:
        #print(f"Error opening serial port: {e}. Please check if the port is correct and not in use.")
        ser = None # Set ser to None if opening fails

    # 加载模型
    try:
        model = joblib.load('D:/备份5/gesture_model.pkl')
    except:
        model = None

    while True:
        success, img = cap.read()
        if not success:
            print("Failed to capture image from camera.")
            break
        img = detector.findHands(img)
        lmList = detector.findPosition(img, draw=True)
        
        if len(lmList) != 0 and model:
            # 提取 21 个点的归一化坐标（与训练时一致）
            h, w = img.shape[:2]
            features = []
            for lm in lmList:
                norm_x = lm[1] / w  # 归一化x
                norm_y = lm[2] / h  # 归一化y
                features.extend([norm_x, norm_y])
            prediction = model.predict([features])
            current_gesture = prediction[0]

            # ========== 手势状态机 ==========
            cTime = time.time()


            # 状态转移逻辑
            if detector.gesture_state == "WAITING":
                # 等待状态：检测到手势则进入确认状态
                if current_gesture is not None:
                    detector.last_gesture = current_gesture
                    detector.hold_start_time = cTime
                    detector.gesture_state = "CONFIRMING"

            elif detector.gesture_state == "CONFIRMING":
                # 确认状态：检查手势是否一致且保持时间足够
                if current_gesture == detector.last_gesture:
                    # 手势一致，检查保持时间
                    if cTime - detector.hold_start_time >= detector.hold_duration:
                        # 保持时间足够，触发
                        detector.confirmed_gesture = current_gesture
                        detector.gesture_state = "TRIGGERED"
                        detector.cooldown_start = cTime
                        print(f"[STATE MACHINE] Gesture CONFIRMED and TRIGGERED: {current_gesture}")
                else:
                    # 手势变化，回到等待状态
                    detector.gesture_state = "WAITING"
                    detector.last_gesture = None
                    detector.hold_start_time = None

            elif detector.gesture_state == "TRIGGERED":
                # 触发状态：进入冷却期
                if cTime - detector.cooldown_start >= detector.cooldown_duration:
                    detector.gesture_state = "WAITING"
                    detector.confirmed_gesture = None

            # 显示状态（用于调试）
            state_display = f"State: {detector.gesture_state}"
            if detector.gesture_state == "CONFIRMING":
                hold_time = cTime - detector.hold_start_time if detector.hold_start_time else 0
                state_display += f" ({hold_time:.2f}/{detector.hold_duration}s)"
            cv2.putText(img, state_display, (10, 100), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)
            cv2.putText(img, f"Gesture: {current_gesture}", (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # ======================================
            
            # 保持原有的串口逻辑
            thumb_tip_x = lmList[4][1]
            thumb_tip_y = lmList[4][2]

            # ========== 自适应EMA滤波 + 二阶离群点剔除 ==========
            alpha = 0.1  # 默认alpha值
            # 初始化EMA滤波状态
            if detector.ema_x is None:
                detector.ema_x = thumb_tip_x
                detector.ema_y = thumb_tip_y
                detector.last_ema_x = thumb_tip_x
                detector.last_ema_y = thumb_tip_y
                smooth_x = thumb_tip_x
                smooth_y = thumb_tip_y
                pred_x = thumb_tip_x
                pred_y = thumb_tip_y
            else:
                # 计算当前帧速度（与上一帧的差值）
                velocity_x = thumb_tip_x - detector.ema_x
                velocity_y = thumb_tip_y - detector.ema_y

                # 保存速度历史
                detector.velocity_x_history.append(abs(velocity_x))
                detector.velocity_y_history.append(abs(velocity_y))
                if len(detector.velocity_x_history) > detector.max_velocity_history:
                    detector.velocity_x_history.pop(0)
                    detector.velocity_y_history.pop(0)

                # 计算平均速度
                avg_velocity_x = sum(detector.velocity_x_history) / len(detector.velocity_x_history)
                avg_velocity_y = sum(detector.velocity_y_history) / len(detector.velocity_y_history)
                avg_velocity = (avg_velocity_x + avg_velocity_y) / 2

                # 根据速度自适应调整alpha
                # 速度越大 -> alpha越大 -> 对原始值响应越快（减少延迟）
                # 速度越小 -> alpha越小 -> 滤波越平滑
                if avg_velocity > detector.velocity_threshold:
                    alpha = detector.alpha_fast
                else:
                    alpha = detector.alpha_slow

                # 一阶离群点检测：单帧坐标突变
                deviation_x = abs(thumb_tip_x - detector.ema_x)
                deviation_y = abs(thumb_tip_y - detector.ema_y)
                outlier_threshold = 50

                if deviation_x > outlier_threshold or deviation_y > outlier_threshold:
                    alpha = alpha * 0.3

                # 二阶离群点检测：速度突变（当前速度远超历史平均）
                # 如果当前速度是历史平均的3倍以上，可能是MediaPipe跳变
                current_velocity = abs(velocity_x) + abs(velocity_y)
                if len(detector.velocity_x_history) >= 3:
                    history_avg = sum(detector.velocity_x_history) + sum(detector.velocity_y_history)
                    history_avg = history_avg / (len(detector.velocity_x_history) + len(detector.velocity_y_history)) * 2
                    if history_avg > 5 and current_velocity > history_avg * 3:
                        alpha = alpha * 0.2  # 极度保守


                # 保存上一帧EMA值（用于预测）
                detector.last_ema_x = detector.ema_x
                detector.last_ema_y = detector.ema_y

                # EMA公式：filtered = alpha * raw + (1 - alpha) * previous_filtered
                detector.ema_x = alpha * thumb_tip_x + (1 - alpha) * detector.ema_x
                detector.ema_y = alpha * thumb_tip_y + (1 - alpha) * detector.ema_y

                smooth_x = int(detector.ema_x)
                smooth_y = int(detector.ema_y)

                # ========== 预测控制 ==========
                # 用当前平滑位置和上一帧平滑位置计算速度
                # predicted = current + velocity = current + (current - last)
                pred_velocity_x = detector.ema_x - detector.last_ema_x
                pred_velocity_y = detector.ema_y - detector.last_ema_y

                # 预测下一帧位置
                pred_x = detector.ema_x + pred_velocity_x
                pred_y = detector.ema_y + pred_velocity_y

                # 预测合理性检测
                pred_deviation_x = abs(pred_x - detector.ema_x)
                pred_deviation_y = abs(pred_y - detector.ema_y)
                pred_deviation = pred_deviation_x + pred_deviation_y

                # 如果预测偏差过大（超过阈值），说明预测不可靠，使用平滑值
                if pred_deviation > detector.prediction_tolerance:
                    pred_x = detector.ema_x
                    pred_y = detector.ema_y
                    detector.use_prediction = False
                else:
                    detector.use_prediction = True

                # 如果速度太小，也不预测（防止静止时噪声放大）
                if abs(pred_velocity_x) < detector.min_velocity_for_prediction and \
                   abs(pred_velocity_y) < detector.min_velocity_for_prediction:
                    pred_x = detector.ema_x
                    pred_y = detector.ema_y

                # 帧计数
                detector.frame_count += 1
                # ================================================

# ========== FuzzyPID参数自适应补偿 ==========
            # 计算误差：预测目标相对于图像中心(320,240)的偏差
            error_x = pred_x - 320
            error_y = pred_y - 240

            # 计算误差变化率de
            curr_time = time.time()
            if detector.last_time is not None:
                dt = curr_time - detector.last_time
                if dt > 0:
                    de_x = (error_x - detector.last_error_x) / dt
                    de_y = (error_y - detector.last_error_y) / dt
                else:
                    de_x = 0
                    de_y = 0
            else:
                de_x = 0
                de_y = 0
            detector.last_time = curr_time

            # FuzzyPID计算：得到自适应调整后的Kp/Kd和控制量u
            u_x, kp_x, kd_x, loss_x = detector.fpid_x.update(error_x, de_x)
            u_y, kp_y, kd_y, loss_y = detector.fpid_y.update(error_y, de_y)

            # 使用预测值 + FuzzyPID补偿作为最终发送值
            if detector.use_fuzzy_compensation:
                # 将FuzzyPID输出的控制量叠加到预测位置
                # u_x/u_y范围约[-50,50]，作为像素偏移补偿
                send_x = int(pred_x + u_x * 0.5)  # 缩放系数0.5避免过大调整
                send_y = int(pred_y + u_y * 0.5)
            else:
                send_x = int(pred_x)
                send_y = int(pred_y)

            # 更新误差历史
            detector.last_error_x = error_x
            detector.last_error_y = error_y

            # 调试信息
            debug_info = f"Err:({error_x:.1f},{error_y:.1f}) de:({de_x:.1f},{de_y:.1f}) Kp:({kp_x:.3f},{kp_y:.3f}) u:({u_x:.2f},{u_y:.2f})"
            cv2.putText(img, debug_info[:60], (10, 130), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)
            # ================================================

            if ser and ser.is_open:
                data = '#'+str(send_x)+'$'+str(send_y)+'\r\n'
                pred_flag = "[PRED]" if detector.use_prediction else "[SMOOTH]"
                fpid_flag = "[FPID]" if detector.use_fuzzy_compensation else ""
                print(f"Raw: ({thumb_tip_x}, {thumb_tip_y}) -> Smooth: ({smooth_x}, {smooth_y}) -> {pred_flag}{fpid_flag} ({send_x}, {send_y}) | {debug_info}")
                try:
                    ser.write(data.encode())
                except serial.SerialException as e:
                    print(f"Error writing to serial port: {e}")
                    ser.close()
                    ser = None

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                    (255, 0, 255), 3)

        cv2.imshow("Image", img)
        key = cv2.waitKey(1)
        if key == ord('s'):
            break
        if key == ord('p'):
            detector.use_fuzzy_compensation = not detector.use_fuzzy_compensation
            print(f"[TOGGLE] FuzzyPID: {'ON' if detector.use_fuzzy_compensation else 'OFF'}")

    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    main()