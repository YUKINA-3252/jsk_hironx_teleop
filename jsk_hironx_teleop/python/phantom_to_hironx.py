#!/usr/bin/env python

from absl import app
from absl import flags

import rospy
import numpy as np
import actionlib
import tf
from scipy.spatial.transform import Rotation as R
from omni_msgs.msg import OmniState, OmniFeedback
from geometry_msgs.msg import (
    Pose, PoseStamped, Point, Quaternion, WrenchStamped)
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Header, Bool
from jsk_hironx_teleop.srv import ControlBilateral,  ControlBilateralResponse

FLAGS = flags.FLAGS

flags.DEFINE_float(
    'vel_scale',
    1.5,
    'Velocicy scale of master/slave. Larger value means'
    'more movement in robot.')

flags.DEFINE_bool(
    'connect_pose',
    True,
    'Set this to connect TouchUSB and Panda position'
    'command from start. Otherwise you have to call rosservice.')
flags.DEFINE_bool(
    'connect_force',
    True,
    'Set this to connect TouchUSB and Panda force'
    'command from start. Otherwise you have to call rosservice.')


# use raw_input for python2 c.f. https://stackoverflow.com/questions/5868506/backwards-compatible-input-calls-in-python
if hasattr(__builtins__, 'raw_input'):
    input = raw_input

def get_status_overlay(moving, arm_name):
    text = OverlayText()
    text.width = 400
    text.height = 600
    text.left = 10 if arm_name == 'larm' else 600
    text.top = 10
    text.text_size = 25
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    on_txt = """<span style="color: red;">ON</span>"""
    off_txt = """<span style="color: white;">OFF</span>"""
    on_or_off = on_txt if moving else off_txt
    text.text = "TouchUSB control {}".format(on_or_off)
    return text


def get_initial_pose():
    r_i = PoseStamped(header=Header(frame_id='rarm_link0'),
                      pose=Pose(position=Point(0.3, 0.0, 0.48),
                                orientation=Quaternion(1.0, 0.0, 0.0, 0.0)))
    l_i = PoseStamped(header=Header(frame_id='larm_link0'),
                      pose=Pose(position=Point(0.3, 0.0, 0.48),
                                orientation=Quaternion(1.0, 0.0, 0.0, 0.0)))
    return r_i, l_i


def get_initial_pose_single(arm_name):
    return PoseStamped(header=Header(frame_id='{}_link0'.format(arm_name)),
                       pose=Pose(position=Point(0.3, 0.0, 0.48),
                                 orientation=Quaternion(1.0, 0.0, 0.0, 0.0)))

class SingleArmHandler(object):

    def __init__(self, robot_id='dual_hironx', arm_name='larm',
                 master_dev_name='left_device', vel_scale=1.0,
                 force_scale=0.1, send_rot=True):
        self.robot_id = robot_id
        self.arm_name = arm_name
        self.side = 'left' if arm_name == 'larm' else 'right'
        self.master_dev_name = master_dev_name
        # [1000Hz] * [m/mm] * [1.0 (scaler)]
        self.vel_scale = vel_scale
        self.force_scale = force_scale
        self.send_rot = send_rot
        # initial rotation of the TouchUSB
        master_initial_q = R.from_quat([-0.5, -0.5, -0.5, 0.5])
        # initial rotation of the HIRONX Arm
        hironx_initial_q = R.from_quat([1.0, 0, 0, 0])
        # conversion from TouchUSB to HIRONX
        self.q_m_p = hironx_initial_q * master_initial_q.inv()
        self.target_force = OmniFeedback()
        self.zero_force = None  # for initialization
        self.moving = False
        self.tf_listener = None
        self.current_frame_id = '{}_end_coords'.format(arm_name)
        self.setup_ros()

    def setup_ros(self):
        dev_topic = "/{}/phantom/state".format(self.master_dev_name)
        force_topic = "/lhsensor" if self.arm_name == 'larm' else "/rhsensor"
        force_pub_topic = "/{}/phantom/force_feedback".format(
            self.master_dev_name)
        pose_target_pub_topic = '{}/{}_target_pose'.format(
            rospy.get_name(), self.arm_name)
        status_overlay_topic = '/{}/{}/status_overlay'.format(
            self.robot_id, self.arm_name)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(
            '{}_JOINT0_Link'.format(self.arm_name.upper()),
            self.current_frame_id,
            rospy.Time(),
            rospy.Duration(3600)
        )

        self.device_state_sub = rospy.Subscriber(
            dev_topic, OmniState, self.device_state_sub)
        self.hironx_force_sub = rospy.Subscriber(
            force_topic, WrenchStamped, self.force_cb)
        self.target_pose = self.set_initial_pose(no_ask=True)

        self.single_target_pose_pub = rospy.Publisher(
            pose_target_pub_topic, PoseStamped, queue_size=1)
        self.force_feedback_pub = rospy.Publisher(
            force_pub_topic, OmniFeedback, queue_size=1)
        self.status_overlay_pub = rospy.Publisher(
            status_overlay_topic, OverlayText, queue_size=1)

    def device_state_sub(self, msg):
        # display status
        self.display_status()
        if msg.locked:  # not move target pose when locked
            self.moving = False
            return
        # update self.target_pose
        self.moving = True
        self.target_pose.pose.position.x += msg.velocity.y * self.vel_scale
        self.target_pose.pose.position.y += -msg.velocity.x * self.vel_scale
        self.target_pose.pose.position.z += msg.velocity.z * self.vel_scale
        if self.send_rot:
            cur_q = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w])
            tar_q = self.q_m_p * cur_q    # apply conversion
            self.target_pose.pose.orientation.x = -tar_q.as_quat()[0]
            self.target_pose.pose.orientation.y = tar_q.as_quat()[1]
            self.target_pose.pose.orientation.z = -tar_q.as_quat()[2]
            self.target_pose.pose.orientation.w = tar_q.as_quat()[3]
        self.target_pose.header.stamp = rospy.Time.now()
        # publish target pose
        self.single_target_pose_pub.publish(self.target_pose)

    def force_cb(self, msg):
        if self.zero_force is None:
            self.zero_force = np.array([msg.wrench.force.x,
                                        msg.wrench.force.y,
                                        msg.wrench.force.z])
        self.target_force.force.x = (msg.wrench.force.y - self.zero_force[0]) * self.force_scale
        self.target_force.force.y = - (msg.wrench.force.x - self.zero_force[1]) * self.force_scale
        self.target_force.force.z = (msg.wrench.force.z - self.zero_force[2]) * self.force_scale

    def apply_target_force(self):
        self.force_feedback_pub.publish(self.target_force)

    def set_initial_pose(self, no_ask=False):
        if not no_ask:
            res = input(
                'Press Enter to set this {} pose as initial position'.format(
                    self.arm_name))
        current_frame = PoseStamped()
        (trans, rot) = self.tf_listener.lookupTransform(
            '{}_JOINT0_Link'.format(self.arm_name.upper()),
            self.current_frame_id,
            rospy.Time(0)
        )
        current_frame.pose.position.x = trans[0]
        current_frame.pose.position.y = trans[1]
        current_frame.pose.position.z = trans[2]
        current_frame.pose.orientation.x = rot[0]
        current_frame.pose.orientation.y = rot[1]
        current_frame.pose.orientation.z = rot[2]
        current_frame.pose.orientation.w = rot[3]
        current_frame.header.stamp = rospy.Time.now()
        current_frame.header.frame_id = '{}_JOINT0_Link'.format(self.arm_name.upper())
        return current_frame

    def display_status(self):
        msg = get_status_overlay(moving=self.moving, arm_name=self.arm_name)
        self.status_overlay_pub.publish(msg)

    def __del__(self):
        rospy.loginfo("Exiting {} handelr".format(self.arm_name))


class DualArmHandler(object):

    def __init__(self, vel_scale=1.0,
                 pose_connecting=False, force_connecting=False):
        rospy.init_node('DualPhantomMaster')
        self.pose_connecting = pose_connecting
        self.force_connecting = force_connecting
        # self.rarm_handler = SingleArmHandler(
        #     arm_name='rarm', master_dev_name='right_device',
        #     vel_scale=vel_scale)
        self.larm_handler = SingleArmHandler(
            arm_name='larm', master_dev_name='left_device',
            vel_scale=vel_scale)
        # self.arms = [self.rarm_handler, self.larm_handler]
        self.arms = [self.larm_handler]
        # self.rarm_target_pub = rospy.Publisher(
        #     '/dual_hironx/rarm_cartesian_impedance_controller/equilibrium_pose',
        #     PoseStamped, queue_size=1)
        self.larm_target_pub = rospy.Publisher(
            '/dual_hironx/larm_cartesian_impedance_controller/equilibrium_pose',
            PoseStamped, queue_size=1)
        self.control_bilateral_srv = rospy.Service(
            '/dual_hironx/control_bilateral',
            ControlBilateral, self.control_bilateral)

    def set_initial_pose(self):
        # self.rarm_handler.target_pose = self.rarm_handler.set_initial_pose(
        #     no_ask=True)
        self.larm_handler.target_pose = self.larm_handler.set_initial_pose(
            no_ask=True)

    def control_bilateral(self, req):
        rospy.loginfo(
            "Applying bilateral connection status, changing in {}...".
            format(req.wait))
        rospy.sleep(req.wait)
        self.pose_connecting = req.pose_connecting
        self.force_connecting = req.force_connecting
        if req.reset_phantom:
            self.set_initial_pose()
        return ControlBilateralResponse(True)

    def loop_call(self):
        pass
        # # send target pose for both arm if connected
        # if self.pose_connecting:
        #     # self.rarm_target_pub.publish(self.rarm_handler.target_pose)
        #     self.larm_target_pub.publish(self.larm_handler.target_pose)
        # else:
        #     self.set_initial_pose()

        # apply force feedback if connected
        # if self.force_connecting:
        #     for arm in self.arms:
        #         arm.apply_target_force()

    def run(self):
        rospy.loginfo("Start looping")
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            # self.loop_call()
            r.sleep()

    def __del__(self):
        rospy.loginfo("Exiting phantom master node")


def main(argv):
    node = DualArmHandler(vel_scale=FLAGS.vel_scale,
                          pose_connecting=FLAGS.connect_pose,
                          force_connecting=FLAGS.connect_force)
    node.run()


if __name__ == '__main__':
    app.run(main)
