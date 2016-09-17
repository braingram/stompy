#!/usr/bin/env python

#import rospy

from ...leg import teensy


global points_reached
points_reached = []


def on_point_reached(point_index):
    global points_reached
    print("Point reached: %s" % point_index.value)
    points_reached.append(point_index.value)


def on_done_moving():
    print("Done moving...")


def connect_to_teensy():
    teensy.mgr.on('point_reached', on_point_reached)
    # TODO do I need this?
    teensy.mgr.on('done_moving', on_done_moving)


def connect():
    connect_to_teensy()
