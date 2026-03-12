#!/usr/bin/env python
# coding=utf-8
import rospy
import serial

def build_mode_cmd(mode=3):
    """
    构造功能触发帧：ID=0xE0，DATA 长度 11
    DATA 区域：
      [CID, CMD0, CMD1, CMD2... CMD9]
      飞行模式选择：CID=0x01，CMD0=0x01，CMD1=模式号，后面填 0
    模式号（CMD1）：
      0: 姿态
      1: 姿态+定高
      2: 定点
      3: 定点+程控
    """
    HEAD, ADDR, ID = b'\xAA', b'\xFF', b'\xE0'
    LEN = b'\x0B'  # 11 字节 DATA
    # 构造 11 字节 DATA
    data = bytes([
        0x01,    # CID: 功能指令类别——飞行模式选择
        0x01,    # CMD0: 模式选择命令
        mode,    # CMD1: 具体模式号（这里 3 → 定点+程控）
    ] + [0x00]*8)  # 剩余 CMD2~CMD9 填 0

    packet = HEAD + ADDR + ID + LEN + data

    # 计算校验
    SUM = sum(packet) & 0xFF
    ADD = 0
    running = 0
    for b in packet:
        running = (running + b) & 0xFF
        ADD     = (ADD + running) & 0xFF

    return packet + bytes([SUM, ADD])

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('mode_switch')
    port = rospy.get_param('~serial_port', '/dev/ttyS7')
    baud = rospy.get_param('~baud_rate', 115200)
    mode = rospy.get_param('~mode', 3)  # 默认为 3：定点+程控

    # 打开串口
    rospy.loginfo("mode_switch: opening serial port %s @ %d", port, baud)
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        rospy.logerr("mode_switch: failed to open serial port %s: %s", port, str(e))
        exit(1)

    # 等待串口稳定
    rospy.sleep(1.0)

    # 构造并发送模式切换帧
    frame = build_mode_cmd(mode)
    try:
        ser.write(frame)
        rospy.loginfo("mode_switch: sent flight mode switch cmd, mode=%d", mode)
    except Exception as e:
        rospy.logerr("mode_switch: failed to write to serial port: %s", str(e))
    finally:
        ser.close()
