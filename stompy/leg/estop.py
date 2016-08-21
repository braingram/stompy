#!/usr/bin/env python

from . import teensy


def from_ros(severity):
    # send stop to teensy
    teensy.trigger('estop', severity)
    # trigger stop callbacks
    pass


def from_teensy(severity):
    # send stop to ros
    # trigger stop callbacks
    pass
