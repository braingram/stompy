#!/usr/bin/env python

import time

import rospy

from ... import leg

from . import estop
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
    estop.connect()
    # setup trajectory action server
    print("node entering loop")
    st = time.time()
    enabled = False
    while not rospy.is_shutdown():
        lt = time.time()
        leg.teensy.com.handle_stream()
        if not enabled:
            print("Sending enable: %s" % lt)
            leg.teensy.lock.acquire(True)
            leg.teensy.mgr.trigger('enable', True)
            leg.teensy.lock.release()
            enabled = True
        if lt - st > 3.:
            leg.teensy.lock.acquire(True)
            print("status: %s" % leg.teensy.mgr.blocking_trigger('status')[0])
            print("heart: %s" % heart.beat.check())
            leg.teensy.lock.release()
            st = lt
        rospy.sleep(0.001)


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
