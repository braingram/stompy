#!/usr/bin/env python

import rospy

from ...leg import teensy


global last_heartbeat
last_heartbeat = None

maximum_heartbeat = 1.0


def connect():
    teensy.mgr.on('heartbeat', new_teensy_heartbeat)


def new_teensy_heartbeat():
    global last_heartbeat
    last_heartbeat = rospy.Time.now()


def check_teensy_heartbeat():
    if last_heartbeat is None:
        return True
    d = rospy.Time.now() - last_heartbeat
    if d.to_secs() > maximum_heartbeat:
        return False
    return True


def send_teensy_heartbeat():
    teensy.mgr.trigger('heartbeat')
