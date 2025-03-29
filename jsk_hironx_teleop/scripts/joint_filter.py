#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class JointFilter:
    def __init__(self):
        self.joint_to_filter = rospy.get_param('~joints_to_filter', [])
        self.filtered_joint_pub = rospy.Publisher('/filtered_joint_states', JointState, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        filtered_msg = JointState()
        filtered_msg.header = msg.header
        for i, name in enumerate(msg.name):
            if name in self.joint_to_filter:
                filtered_msg.name.append(name)
                filtered_msg.position.append(msg.position[i])
                filtered_msg.velocity.append(msg.velocity[i])
                filtered_msg.effort.append(msg.effort[i])

        if len(filtered_msg.position) != 0:
            self.filtered_joint_pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('joint_filter')
    jf = JointFilter()
    rospy.spin()
