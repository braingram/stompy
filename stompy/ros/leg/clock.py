#!/usr/bin/env python

import rospy

from ... import leg

global offset
offset = 0.


def measure_offset():
    """Measure offset of ros time from teensy time

    to convert from ros time to teensy time:
       (rt.to_sec() + offset) / 1000.
    """
    global offset
    t1 = rospy.Time.now().to_sec()
    t1p = leg.teensy.mgr.blocking_trigger('get_time')[0].value / 1000.
    t2 = leg.teensy.mgr.blocking_trigger('get_time')[0].value / 1000.
    t2p = rospy.Time.now().to_sec()
    offset = (t1p - t1 - t2p + t2) / 2.
    print("Clocks synchronized: %s, %s, %s, %s" % (t1, t1p, t2, t2p))
    return offset


def convert_ros_time(ros_time):
    return int((ros_time.to_sec() + offset) * 1000.)


def convert_teensy_time(teensy_time):
    return rospy.Time(teensy_time / 1000. - offset)
