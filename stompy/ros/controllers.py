#!/usr/bin/env python

import control_msgs.msg
import rospy

from ..sensors import controllers


def new_joint_states(msg, name):
    controllers.update_state(name, msg)


def connect_to_leg_controllers():
    for jn in ('fr', 'fl', 'mr', 'ml', 'rr', 'rl'):
        rospy.Subscriber(
            "/stompy/%s/state" % jn,
            control_msgs.msg.JointTrajectoryControllerState,
            lambda msg, n=jn: new_joint_states(msg, n))
