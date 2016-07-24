#!/usr/bin/env python

#import rospy
import actionlib

import control_msgs.msg

from .. import info


global publishers
publishers = None


def connect_to_leg_publishers(legs=None):
    if legs is None:
        legs = info.legs
    global publishers
    publishers = {}
    for leg_name in legs:
        publishers[leg_name] = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % leg_name,
            control_msgs.msg.FollowJointTrajectoryAction)
    return publishers
