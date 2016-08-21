#!/usr/bin/env python

import time

import rospy

import heartbeat.heart

from ...leg import teensy

from . import info


global beat
beat = None

global last_heartbeat
last_heartbeat = None

maximum_heartbeat = 1.0

global foo
foo = 0


def connect_to_ros():
    global beat
    print("heart connecting to ros")
    beat = heartbeat.heart.ClientHeart(info.name, 'head')


def connect_to_teensy():
    print("heart connecting to teensy")
    teensy.mgr.on('heartbeat', new_teensy_heartbeat)
    if beat is not None:
        print("attaching ros heartbeat to teensy")
        # callback, when ros gets hb, send to teensy
        beat.attach(lambda hb: send_teensy_heartbeat())


def connect():
    connect_to_ros()
    connect_to_teensy()


def new_teensy_heartbeat():
    global last_heartbeat
    print("received teensy heartbeat: %.4f" % (time.time() - foo))
    last_heartbeat = rospy.Time.now()


def check_teensy_heartbeat():
    if last_heartbeat is None:
        return True
    d = rospy.Time.now() - last_heartbeat
    if d.to_secs() > maximum_heartbeat:
        return False
    return True


def send_teensy_heartbeat():
    global foo
    foo = time.time()
    print("sending teensy heartbeat: %.4f" % foo)
    teensy.mgr.trigger('heartbeat')


# TODO check
