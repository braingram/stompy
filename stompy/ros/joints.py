#!/usr/bin/env python

import rospy
import sensor_msgs.msg

from ..sensors import joints


def connect_to_joint_states():
    rospy.Subscriber(
        "/stompy/joint_states", sensor_msgs.msg.JointState,
        lambda data, description=joints.stompy_leg_descriptions:
        joints.update_joints(data, description))
