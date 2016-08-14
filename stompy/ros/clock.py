#!/usr/bin/env python


from ..world import clock

import rospy
import rosgraph_msgs.msg


def new_clock_tick(msg):
    clock.update_time(msg.clock.secs + msg.clock.nsecs / 1.0E9)


def connect():
    rospy.Subscriber('clock', rosgraph_msgs.msg.Clock, new_clock_tick)
