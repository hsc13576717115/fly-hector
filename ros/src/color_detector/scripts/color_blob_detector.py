#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorBlobDetector:
    def __init__(self):
        rospy.init_node("color_blob_detector")
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ¶¨ÒåÑÕÉ«·¶Î§
        colors = {
            "Red": [(0, 120, 70), (10, 255, 255)],
            "Yellow": [(20, 100, 100), (30, 255, 255)],
            "Blue": [(100, 150, 50), (140, 255, 255)]
        }

        for name, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    rospy.loginfo(f"Detected {name} color at ({x}, {y}), area: {area}")

        cv2.imshow("Color Blobs", frame)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = ColorBlobDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
