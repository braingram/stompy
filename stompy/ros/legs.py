#!/usr/bin/env python

#import rospy
import actionlib

import control_msgs.msg


default_legs = ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']

global publishers
publishers = None


def connect_to_leg_publishers(legs=None):
    if legs is None:
        legs = default_legs
    global publishers
    publishers = {}
    for leg_name in legs:
        publishers[leg_name] = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % leg_name,
            control_msgs.msg.FollowJointTrajectoryAction)
    return publishers
