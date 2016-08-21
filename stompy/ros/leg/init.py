#!/usr/bin/env python

import rospy

from ... import leg

from . import heart
from . import info
from . import joints


def init_leg(name=None):
    if name is None:
        name = info.name
    else:
        info.name = name
    leg.teensy.connect()
    print("making ros node")
    rospy.init_node(name, anonymous=True)
    heart.connect()
    joints.connect()
    # setup trajectory action server
    print("node entering loop")
    while not rospy.is_shutdown():
        leg.teensy.com.handle_stream()
        print("heart: %s" % heart.beat.check())
        rospy.sleep(0.01)


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
