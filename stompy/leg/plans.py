#!/usr/bin/env python
"""
Some example plans:
    1) move hip[X] joint at 1500 sensor coords / sec:
        [1, 0, 1., 0., 0., 0., 0., 0., 1500]
    2) move foot in X at 10 inches / sec
        [1, 2, 1., 0., 0., 0., 0., 0., 10.]
    3) move foot to 20, 0, -15 at 10 inches / sec
        [3, 2, 20, 0, -15, 0., 0., 0., 10.]
"""

from . import consts


class Plan(object):
    def __init__(
            self, mode=consts.PLAN_STOP_MODE, frame=consts.PLAN_SENSOR_FRAME,
            linear=None, angular=None, speed=0.):
        self.mode = mode
        self.frame = frame
        self.linear = linear
        self.angular = angular
        self.speed = speed

    def packed(self):
        if self.linear is None:
            l = (0., 0., 0.)
        else:
            l = self.linear
        if self.angular is None:
            a = (0., 0., 0.)
        else:
            a = self.angular
        return [
            self.mode, self.frame,
            l[0], l[1], l[2],
            a[0], a[1], a[2],
            self.speed]


def stop():
    return Plan(consts.PLAN_STOP_MODE)
