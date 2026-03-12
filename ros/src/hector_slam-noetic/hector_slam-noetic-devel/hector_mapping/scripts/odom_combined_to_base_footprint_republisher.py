#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

"""
This node continuously republishes a static transform from odom_combined to base_footprint
with current timestamps so that move_base always gets a fresh TF.
"""

def main():
    rospy.init_node('odom_combined_to_base_footprint_republisher')
    br = tf2_ros.TransformBroadcaster()
    rate_hz = rospy.get_param('~rate', 50.0)  # default 50Hz
    rate = rospy.Rate(rate_hz)
    # translation and rotation parameters (defaults: zero translation, identity rotation)
    trans = rospy.get_param('~translation', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    rot = rospy.get_param('~rotation', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
    parent_frame = rospy.get_param('~parent_frame', 'odom_combined')
    child_frame = rospy.get_param('~child_frame', 'base_footprint')

    while not rospy.is_shutdown():
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = trans['x']
        t.transform.translation.y = trans['y']
        t.transform.translation.z = trans['z']
        t.transform.rotation.x = rot['x']
        t.transform.rotation.y = rot['y']
        t.transform.rotation.z = rot['z']
        t.transform.rotation.w = rot['w']
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
