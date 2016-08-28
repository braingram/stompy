#!/usr/bin/env python

import rospy

from ... import leg

from . import estop
from . import heart
from . import info
from . import joints


def measure_clock_offset():
    """Measure offset of ros time from teensy time

    to convert from ros time to teensy time:
       (rt.to_sec() + offset) / 1000.
    """
    t1 = rospy.Time.now().to_sec()
    t1p = leg.teensy.mgr.blocking_trigger('get_time')[0].value / 1000.
    t2 = leg.teensy.mgr.blocking_trigger('get_time')[0].value / 1000.
    t2p = rospy.Time.now().to_sec()
    offset = (t1p - t1 - t2p + t2) / 2.
    return offset


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
    st = rospy.Time.now()
    enabled = False
    offset = None
    while not rospy.is_shutdown():
        lt = rospy.Time.now()
        leg.teensy.com.handle_stream()
        if not enabled:
            print("Sending enable: %s" % lt)
            leg.teensy.lock.acquire(True)
            leg.teensy.mgr.trigger('enable', True)
            leg.teensy.lock.release()
            enabled = True
        if lt - st > rospy.Duration(3.):
            leg.teensy.lock.acquire(True)
            print("status: %s" % leg.teensy.mgr.blocking_trigger('status')[0])
            print("heart: %s" % heart.beat.check())
            new_offset = measure_clock_offset()
            if offset is not None:
                print("offset: %0.4f, delta: %s" % (new_offset, offset - new_offset))
            offset = new_offset
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
