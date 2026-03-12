#!/usr/bin/env python3
import rospy
import serial
import roslaunch

def main():
    rospy.init_node('serial_and_launcher')

    # 参数读取
    port           = rospy.get_param('~serial_port',   '/dev/ttyS7')
    baud           = rospy.get_param('~baud_rate',     115200)
    unlock_cmd_id  = rospy.get_param('~unlock_cmd_id', 17)    # 17 表示解锁
    takeoff_cmd_id = rospy.get_param('~takeoff_cmd_id',16)    # 16 表示起飞
    hover_time     = rospy.get_param('~hover_time',     1)    # 悬停时长，单位秒
    post_cmd_id    = rospy.get_param('~post_cmd_id',     5)   # 悬停后发送的命令ID
    post_cmd_val   = rospy.get_param('~post_cmd_val',    0)   # 悬停后命令的参数
    post_duration  = rospy.get_param('~post_duration',   3)   # 悬停后命令持续时长，单位秒
    next_launch    = rospy.get_param('~next_launch')

    rospy.loginfo(f"[serial_and_launcher] Opening serial {port}@{baud}")
    ser = serial.Serial(port, baud, timeout=1.0)

    # ---- 1) 先发送“解锁”命令 ----
    unlock_str = f"<{unlock_cmd_id} 0>\r\n"
    rospy.loginfo(f"[serial_and_launcher] Sending unlock: {unlock_str.strip()}")
    ser.write(unlock_str.encode('ascii'))

    # 给予飞控一点时间处理解锁
    rospy.sleep(1.0)

    # ---- 2) 再发送“一键起飞+悬停”命令 ----
    takeoff_str = f"<{takeoff_cmd_id} {hover_time}>\r\n"
    rospy.loginfo(f"[serial_and_launcher] Sending takeoff/hover: {takeoff_str.strip()}")
    ser.write(takeoff_str.encode('ascii'))

    # 悬停指定时长
    rospy.loginfo(f"[serial_and_launcher] Hovering for {hover_time} seconds...")
    rospy.sleep(hover_time)

    # ---- 3) 悬停后发送指定命令 ----
    post_str = f"<{post_cmd_id} {post_cmd_val}>\r\n"
    rospy.loginfo(f"[serial_and_launcher] Sending post-hover command: {post_str.strip()}")
    ser.write(post_str.encode('ascii'))
    rospy.loginfo(f"[serial_and_launcher] Holding post command for {post_duration} seconds...")
    rospy.sleep(post_duration)

    # ---- 4) 启动 phase2.launch ----
    rospy.loginfo(f"[serial_and_launcher] Launching next: {next_launch}")
    uuid     = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launcher = roslaunch.parent.ROSLaunchParent(uuid, [next_launch])
    launcher.start()

    rospy.loginfo("[serial_and_launcher] phase2 launched, now spinning.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass