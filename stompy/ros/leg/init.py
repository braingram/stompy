#!/usr/bin/env python

import rospy

from ... import leg

from . import heart
from . import info
from . import joints


def init_leg(name=None):
    if name is None:
        name = info.name
    leg.teensy.connect()
    rospy.init_node(name, anonymous=True)
    heart.connect()
    joints.connect()
    # setup trajectory action server
    while not rospy.is_shutdown():
        heart.send_teensy_heartbeat()
        rospy.sleep(0.5)


def fake_joints(name=None):
    if name is None:
        name = info.name
    # this will block until the master is up
    rospy.init_node(name + '_fake_joints', anonymous=True)
    joints.connect_to_ros()
    heart.connect_to_ros()
    while not rospy.is_shutdown():
        print("sending joints...")
        joints.send_joints()
        print("heart: %s" % heart.beat.check())
        rospy.sleep(0.1)
