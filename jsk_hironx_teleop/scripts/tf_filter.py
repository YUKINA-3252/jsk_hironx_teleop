#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def callback(trans):
    rospy.loginfo("Received Transform: {trans}")

def main():
    rospy.init_node('tf_listener_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('world', 'larm_end_coords', rospy.Time())
            callback(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
        rate.sleep()

if __name__ == '__main__':
    main()
