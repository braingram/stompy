#!/usr/bin/env python

import rospy

import std_msgs.msg

from ... import leg

global pub
pub = None


def on_estop(msg):
    severity = msg.data
    leg.teensy.lock.acquire(True)
    leg.teensy.mgr.trigger('estop', severity)
    leg.teensy.lock.release()
    # TODO other estop callbacks


def connect_to_ros():
    rospy.Subscriber('/estop', std_msgs.msg.Byte, on_estop)
    global pub
    pub = rospy.Publisher('/estop', std_msgs.msg.Byte, queue_size=10)


def new_teensy_estop(severity):
    msg = std_msgs.msg.Byte()
    msg.data = severity.value
    pub.publish(msg)
    # TODO other estop callbacks


def connect_to_teensy():
    leg.teensy.mgr.on('estop', new_teensy_estop)


def connect():
    connect_to_ros()
    connect_to_teensy()
