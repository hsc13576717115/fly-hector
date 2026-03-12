#!/usr/bin/env python
import rospy, serial, struct
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# 帧头定义（根据凌霄协议修改）
FRAME_HEADER = b'\xAA\xFF'  # 协议中的帧头标识
CMD_ID_IMU = 0x01           # IMU数据帧ID

def parse_imu_frame(pkt):
    # 根据协议解析数据帧（参考凌霄手册数据格式）
    # 假设数据格式为：ax(2B), ay(2B), az(2B), gx(2B), gy(2B), gz(2B), checksum(1B)
    # 实际格式需根据飞控协议文档调整
    ax, ay, az, gx, gy, gz, checksum = struct.unpack('<hhhhhhB', pkt)
    
    # 校验和验证（示例为简单累加和校验）
    calc_sum = sum(pkt[:-1]) % 256
    if calc_sum != checksum:
        rospy.logwarn("Checksum failed")
        return None

    imu = Imu()
    imu.header = Header()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'imu_link'  # 建议使用独立坐标系
    
    # 单位转换（根据实际传感器参数调整）
    imu.linear_acceleration.x = ax * 9.8 / 1000.0  # 假设原始数据单位为mg
    imu.linear_acceleration.y = ay * 9.8 / 1000.0
    imu.linear_acceleration.z = az * 9.8 / 1000.0
    
    imu.angular_velocity.x = gx * 0.0174533  # 假设原始数据单位为0.01dps
    imu.angular_velocity.y = gy * 0.0174533
    imu.angular_velocity.z = gz * 0.0174533

    return imu

def imu_publisher():
    rospy.init_node('imu_serial_node')
    port = rospy.get_param('~port', '/dev/ttyS3')
    baud = rospy.get_param('~baud', 115200)
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port: {e}")
        return

    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    buffer = bytearray()
    rate = rospy.Rate(200)  # 提高读取频率
    
    while not rospy.is_shutdown():
        # 读取所有可用字节
        buffer += ser.read(ser.in_waiting or 1)
        
        # 帧头检测
        while len(buffer) >= 2 and buffer[0:2] == FRAME_HEADER:
            # 检查数据长度
            if len(buffer) < 4:
                break
                
            frame_id = buffer[2]
            length = buffer[3]
            
            # 检查完整帧
            if len(buffer) < 4 + length + 1:  # 包含校验位
                break
                
            # 提取完整数据帧
            frame = buffer[0:4+length+1]
            buffer = buffer[4+length+1:]
            
            # 处理IMU数据帧
            if frame_id == CMD_ID_IMU:
                imu_msg = parse_imu_frame(frame[4:-1])  # 去掉帧头和校验位
                if imu_msg:
                    pub.publish(imu_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass