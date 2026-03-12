#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import signal  # 新增：信号处理模块
from std_msgs.msg import String
import cv2
import numpy as np
import serial
from threading import Event  # 新增：用于控制循环的事件


# 设定目标颜色（可改为"Red2", "Green", "Blue"）
TARGET_COLOR = "Red2"  
# TARGET_COLOR = "Green"  # 示例：绿色触发

# 新增：用于控制主循环的事件（解决cv2.waitKey阻塞信号的问题）
shutdown_event = Event()  


def send_off(ser):
    """ 发送降落指令 """
    data_str = "<67 0>\r\n"
    ser.write(data_str.encode())
    rospy.loginfo(f"[ColorBlob] 检测到{TARGET_COLOR}色块，开始降落！")


def detect_colors(frame, xy_tolerance):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colors = {
        "Red1": [(0, 120, 70), (10, 255, 255)],
        "Red2": [(160, 120, 70), (180, 255, 255)],
        "Green": [(40, 40, 40), (80, 255, 255)],
        "Blue": [(100, 150, 50), (140, 255, 255)]
    }

    detected_blocks = []
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2 -30

    for name, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 80000:
                x, y, w, h = cv2.boundingRect(cnt)
                rect_center_x = x + w // 2
                rect_center_y = y + h // 2
                offset_x = (center_y - rect_center_y)  # X正=上，X负=下
                offset_y = (center_x - rect_center_x)  # Y正=左，Y负=右

                detected_blocks.append({
                    "area": area,
                    "offset_x": offset_x,
                    "offset_y": offset_y,
                    "name": name  # 保留颜色名称
                })

                # 绘制色块信息（显示颜色和坐标）
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, f"{name}: (X={offset_x}, Y={offset_y})", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 绘制中心和容忍度圆圈
    cv2.drawMarker(frame, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.circle(frame, (center_x, center_y), xy_tolerance, (0, 255, 0), 2)
    cv2.putText(frame, f"Tol: {xy_tolerance}", (center_x+10, center_y+20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return detected_blocks, frame  # 返回所有色块（包含颜色信息）


def signal_handler(signum, frame):
    """ 自定义信号处理函数：触发shutdown事件 """
    rospy.loginfo("[ColorBlob] 检测到Ctrl+C，准备退出...")
    shutdown_event.set()  # 触发事件，通知主循环退出
    rospy.signal_shutdown("User requested shutdown")  # 通知rospy关闭


def main():
    rospy.init_node("color_blob_ros_node")
    pub = rospy.Publisher("/color_detect_result", String, queue_size=10)

    # 新增：注册信号处理（覆盖默认的rospy信号处理）
    signal.signal(signal.SIGINT, signal_handler)  

    serial_port = rospy.get_param('~serial_port', '/dev/ttyS7')
    baud_rate = rospy.get_param('~baud_rate', 115200)
    xy_tolerance = 30  # 像素容忍度
    max_speed = 5       # cm/s
    Kp_x = 0.03
    Kp_y = 0.03

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        rospy.loginfo(f"[ColorBlob] 目标颜色设定为：{TARGET_COLOR}，串口连接成功：{serial_port} @ {baud_rate}")
    except Exception as e:
        rospy.logfatal(f"串口初始化失败：{e}")
        return

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("摄像头打开失败")
        return

    rospy.loginfo(f"[ColorBlob] 开始检测{TARGET_COLOR}色块...")
    rate = rospy.Rate(10)

    # 主循环改为监听shutdown_event和rospy.is_shutdown()
    while not shutdown_event.is_set() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            ser.write("<0 0>\r\n".encode())
            rospy.logwarn("摄像头帧读取失败")
            continue

        all_blocks, display_frame = detect_colors(frame, xy_tolerance)
        target_block = None

        # 筛选目标颜色的色块（取面积最大的）
        target_blocks = [b for b in all_blocks if b["name"] == TARGET_COLOR]
        if target_blocks:
            target_block = max(target_blocks, key=lambda x: x["area"])

        if target_block:
            offset_x = target_block["offset_x"]
            offset_y = target_block["offset_y"]
            rospy.logdebug(f"{TARGET_COLOR}色块偏移：(X={offset_x}, Y={offset_y})")

            # 仅当目标颜色在中心时触发降落
            if abs(offset_x) <= xy_tolerance and abs(offset_y) <= xy_tolerance:
                ser.write("<0 0>\r\n".encode())
                ser.write("<0 0>\r\n".encode())
                ser.write("<0 0>\r\n".encode())
                send_off(ser)
                send_off(ser)
                send_off(ser)
                break  # 降落后不再发送速度指令

            # 计算速度（仅对目标颜色生效）
            vx = Kp_x * offset_x
            vy = Kp_y * offset_y
            vx = np.clip(vx, -max_speed, max_speed)
            vy = np.clip(vy, -max_speed, max_speed)
            ser.write(f"<{int(vx)} {int(vy)}>\r\n".encode())
            rospy.loginfo(f"调整中：{TARGET_COLOR}色块，vx={vx}, vy={vy}")

        else:
            ser.write("<0 0>\r\n".encode())
            rospy.loginfo(f"未检测到{TARGET_COLOR}色块，悬停")

        cv2.imshow("Detected Colors", display_frame)
        # 新增：使用waitKey(1)并检查事件（避免阻塞信号）
        if cv2.waitKey(1) == 27 or shutdown_event.is_set():
            break

        rate.sleep()

    # 清理资源（确保执行）
    rospy.loginfo("[ColorBlob] 正在清理资源...")
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()
    if ser.is_open:
        ser.close()
    rospy.loginfo("[ColorBlob] 节点已成功退出")


if __name__ == "__main__":
    main()