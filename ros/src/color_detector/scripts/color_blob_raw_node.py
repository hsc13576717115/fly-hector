#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import cv2
import numpy as np

def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ¶¨ÒåÑÕÉ«·¶Î§
    colors = {
        "Red1": [(0, 120, 70), (10, 255, 255)],
        "Red2": [(160, 120, 70), (180, 255, 255)],
        "Green": [(40, 40, 40), (80, 255, 255)],
        "Blue": [(100, 150, 50), (140, 255, 255)]
    }

    result = []

    for name, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 80000:
                x, y, w, h = cv2.boundingRect(cnt)
                result.append(f"{name} at ({x},{y}), area={area}")
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(frame, name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    return result, frame

def main():
    rospy.init_node("color_blob_ros_node")
    pub = rospy.Publisher("/color_detect_result", String, queue_size=10)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Failed to open camera /dev/video0")
        return

    rospy.loginfo("Color detection node started.")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        results, display_frame = detect_colors(frame)
        for r in results:
            rospy.loginfo(r)
            pub.publish(r)

        cv2.imshow("Detected Colors", display_frame)
        if cv2.waitKey(1) == 27:  # ESC to quit
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
