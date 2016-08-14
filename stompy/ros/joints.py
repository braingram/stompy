#!/usr/bin/env python

import rospy
import sensor_msgs.msg

from ..sensors import joints


def new_joint_states(data):
    joints.update_joints(
        dict(zip(data.name, data.position)),
        data.header.stamp.secs + data.header.stamp.nsecs / 1.E9)


def connect_to_joint_states():
    rospy.Subscriber(
        "/stompy/joint_states", sensor_msgs.msg.JointState,
        new_joint_states)
