# -*- coding: utf-8 -*-
#!/usr/bin/env python



import rospy
import serial
import math
import tf
import subprocess
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatus, GoalStatusArray, GoalID
import tf2_ros
import tf2_geometry_msgs

# 新增色块检测依赖
import cv2
import numpy as np

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        # 参数初始化
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyS7')
        self.baud_rate   = rospy.get_param('~baud_rate', 115200)
        self.max_speed   = rospy.get_param('~max_speed', 10)  # cm/s
        self.hz          = rospy.get_param('~hz', 10.0)
        self.plan_topic  = rospy.get_param('~plan_topic', '/move_base/DWAPlannerROS/local_plan')
        self.waypoints = [
            (rospy.get_param('~a_x', 2.0), rospy.get_param('~a_y', 0.0), 0.0),
            (rospy.get_param('~b_x', 2.0), rospy.get_param('~b_y', 1.0), 0.0),
            (rospy.get_param('~c_x', 3.0), rospy.get_param('~c_y', 0.0), 0.0),  # 第3个点，索引2
            (rospy.get_param('~d_x', 3.0), rospy.get_param('~d_y', 1.2), 0.0),
            (rospy.get_param('~e_x', 3.3), rospy.get_param('~e_y', 1.2), 0.0),
            (rospy.get_param('~f_x', 3.3), rospy.get_param('~f_y', 0.0), 0.0),
            (rospy.get_param('~g_x', 3.3), rospy.get_param('~g_y', 0.0), 0.0),
        ]
        self.current_wp         = 0
        self.hover_time         = rospy.get_param('~hover_time', 2.0)
        self.hover_time_wp2     = rospy.get_param('~hover_time_wp2', 5.0)
        self.xy_tolerance       = rospy.get_param('~xy_tolerance', 0.2)
        self.hover_start        = None
        self.is_hovering        = False
        self.goal_processed     = False
        self.all_goals_completed = False

        # Publisher and TF
        self.goal_pub   = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.tf_buffer  = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Serial init
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            rospy.loginfo(f"[PathFollower] 串口连接：{self.serial_port} @ {self.baud_rate}")
        except Exception as e:
            rospy.logfatal(f"[PathFollower] 串口初始化失败：{e}")
            raise

        # Subscribers and timer
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.cb_goal_status)
        rospy.Subscriber(self.plan_topic, Path, self.cb_path)
        rospy.Timer(rospy.Duration(1.0 / self.hz), self.check_hover)

        rospy.loginfo("[PathFollower] 初始化完成，开始发送第一个目标")
        rospy.sleep(2.0)
        self.send_goal(self.current_wp)
        rospy.spin()

    def send_goal(self, idx):
        rospy.loginfo(f"[PathFollower] 准备发送第 {idx} 个 waypoint")
        if idx < len(self.waypoints):
            cancel = GoalID(); cancel.stamp = rospy.Time.now(); cancel.id = ''
            self.cancel_pub.publish(cancel)
            rospy.sleep(0.2)
            self.send_dummy_goal()
            rospy.sleep(0.2)

            x, y, yaw = self.waypoints[idx]
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))
            self.goal_pub.publish(ps)
            rospy.loginfo(f"[PathFollower] 发送目标 WP{idx}： x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        else:
            cancel = GoalID(); cancel.stamp = rospy.Time.now(); cancel.id = ''
            self.cancel_pub.publish(cancel)
            self.send_zero_velocity()
            self.all_goals_completed = True
            rospy.loginfo("[PathFollower] 所有waypoints完成，节点退出")

    def send_dummy_goal(self):
        current = self.get_current_pose_in_map()
        if current is None:
            return
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = current.position.x
        ps.pose.position.y = current.position.y
        ps.pose.position.z = current.position.z
        ps.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.goal_pub.publish(ps)

    def cb_goal_status(self, msg):
        if self.all_goals_completed or self.goal_processed:
            return
        for s in msg.status_list:
            if s.status == GoalStatus.SUCCEEDED:
                pose = self.get_current_pose_in_map()
                if pose is None:
                    return
                tx, ty, _ = self.waypoints[self.current_wp]
                dist = math.hypot(tx - pose.position.x, ty - pose.position.y)
                if dist <= self.xy_tolerance:
                    self.hover_start    = rospy.get_time()
                    self.is_hovering    = True
                    self.goal_processed = True
                    self.send_zero_velocity()
                else:
                    self.is_hovering = False

    def check_hover(self, event):
        if not self.is_hovering:
            return
        elapsed = rospy.get_time() - self.hover_start
        # 根据当前 waypoint 选择悬停时长
        target_hover = self.hover_time_wp2 if self.current_wp == 2 else self.hover_time
        rospy.loginfo(f"[PathFollower] 当前悬停时间：{elapsed:.2f}s / 目标悬停时间：{target_hover:.2f}s")
        if elapsed >= target_hover:
            self.is_hovering = False
            self.goal_processed = False

            # 在第3个目标（索引2）悬停完成后开启色块识别
            if self.current_wp == 2:
                self.detect_color_once()

            # 切换下一个目标
            self.current_wp += 1
            if self.current_wp < len(self.waypoints):
                self.send_goal(self.current_wp)
            else:
                rospy.loginfo("[PathFollower] 全部waypoints执行完毕，启动phase3.launch")
                subprocess.call(['rosnode', 'kill', '/move_base'])
                rospy.sleep(1.0)

                subprocess.Popen(['roslaunch', 'hector_mapping', 'phase3.launch'])
                rospy.signal_shutdown('任务完成')

    def cb_path(self, msg):
        if self.all_goals_completed or self.is_hovering:
            return
        pose = self.get_current_pose_in_map()
        if pose is None:
            return
        tx, ty, _ = self.waypoints[self.current_wp]
        dist = math.hypot(tx - pose.position.x, ty - pose.position.y)
        if dist > self.xy_tolerance:
            self.send_path_velocity(msg)
        else:
            self.hover_start    = rospy.get_time()
            self.is_hovering    = True
            self.goal_processed = True
            self.send_zero_velocity()

    def get_current_pose_in_map(self):
        try:
            tfm = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            ps = PoseStamped(header=tfm.header, pose=Pose())
            return tf2_geometry_msgs.do_transform_pose(ps, tfm).pose
        except Exception:
            return None

    def send_path_velocity(self, msg):
        poses = msg.poses
        if len(poses) < 2:
            self.send_zero_velocity()
            return
        p0, p1 = poses[0].pose.position, poses[1].pose.position
        dt = 1.0 / self.hz
        vx = (p1.x - p0.x) / dt * 100
        vy = (p1.y - p0.y) / dt * 100
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale; vy *= scale
        cmd = f"<{int(vx)} {int(vy)}>\r\n"
        try:
            self.ser.write(cmd.encode())
        except Exception:
            pass

    def send_zero_velocity(self):
        try:
            self.ser.write(b"<0 0>\r\n")
        except Exception:
            pass

    # 新增：色块识别一次性函数
    def detect_color_once(self):
        """
        读取摄像头10帧，每帧检测并打印色块结果。
        """
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("[ColorBlob] 摄像头打开失败，无法检测色块")
            return
        rospy.loginfo("[ColorBlob] 开始检测10帧色块...")
        try:
            count = 0
            while count < 20 and not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    rospy.logwarn("[ColorBlob] 读取帧失败，重试...")
                    continue

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                colors = {
                    "Red1": [(0, 120, 70), (10, 255, 255)],
                    "Red2": [(160, 120, 70), (180, 255, 255)],
                    "Green": [(40, 40, 40), (80, 255, 255)],
                    "Blue": [(100, 150, 50), (140, 255, 255)]
                }
                detected = []
                for name, (lower, upper) in colors.items():
                    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in contours:
                        if cv2.contourArea(cnt) > 80000:
                            detected.append(name)

                if detected:
                    # 统计出现频次最多的颜色
                    color = max(set(detected), key=detected.count)
                    rospy.loginfo(f"[ColorBlob] 帧{count+1}: 检测到色块：{color}")
                else:
                    rospy.loginfo(f"[ColorBlob] 帧{count+1}: 未检测到大型色块")

                count += 1
        finally:
            cap.release()
            rospy.loginfo("[ColorBlob] 完成10帧检测，已释放摄像头")

if __name__ == '__main__':
    try:
        PathFollower()
    except rospy.ROSInterruptException:
        rospy.loginfo("[PathFollower] 节点退出")
