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

import numpy

from .. import consts
from .. import kinematics


class Plan(object):
    def __init__(
            self, mode=consts.PLAN_STOP_MODE, frame=consts.PLAN_SENSOR_FRAME,
            linear=None, angular=None, matrix=None, speed=0.):
        self.mode = mode
        self.frame = frame
        self.linear = linear
        self.angular = angular
        self.speed = speed
        self.matrix = matrix

    def packed(self, leg_number):
        # convert from body to leg
        if self.linear is None:
            l = (0., 0., 0.)
        else:
            l = self.linear
        if self.angular is None:
            a = (0., 0., 0.)
        else:
            a = self.angular
        f = self.frame
        m = numpy.matrix(self.matrix)
        if f == consts.PLAN_BODY_FRAME:
            if leg_number in kinematics.body.body_to_leg_transforms:
                # convert from body to leg
                if self.mode == consts.PLAN_STOP_MODE:
                    # stop: do nothing
                    pass
                elif self.mode == consts.PLAN_TARGET_MODE:
                    # target: convert linear
                    l = kinematics.body.body_to_leg(
                        leg_number, l[0], l[1], l[2])
                elif self.mode == consts.PLAN_VELOCITY_MODE:
                    # vel: convert linear as vector, just rotate
                    l = kinematics.body.body_to_leg_rotation(
                        leg_number, l[0], l[1], l[2])
                elif self.mode == consts.PLAN_ARC_MODE:
                    # arc: raise NotImplementedError()
                    # linear: translate to leg
                    # angular: rotate to leg
                    # speed: keep the same
                    l = kinematics.body.body_to_leg(
                        leg_number, l[0], l[1], l[2])
                    a = kinematics.body.body_to_leg_rotation(
                        leg_number, a[0], a[1], a[2])
                elif self.mode == consts.PLAN_MATRIX_MODE:
                    # combine with body transform
                    m *= kinematics.body.body_to_leg_transforms[leg_number]
            f = consts.PLAN_LEG_FRAME
        if self.mode == consts.PLAN_STOP_MODE:
            return [self.mode, f, self.speed]
        if self.mode in (consts.PLAN_TARGET_MODE, consts.PLAN_VELOCITY_MODE):
            return [
                self.mode, f, l[0], l[1], l[2], self.speed]
        if self.mode == consts.PLAN_ARC_MODE:
            return [
                self.mode, f,
                l[0], l[1], l[2],
                a[0], a[1], a[2],
                self.speed]
        if self.mode == consts.PLAN_MATRIX_MODE:
            # don't send last row, assuming this is always 0, 0, 0, 1
            # I think comando has a bug with >64 byte messages
            return [
                self.mode, f,
                m[0, 0], m[0, 1], m[0, 2], m[0, 3],
                m[1, 0], m[1, 1], m[1, 2], m[1, 3],
                m[2, 0], m[2, 1], m[2, 2], m[2, 3],
                #m[3, 0], m[3, 1], m[3, 2], m[3, 3],
                self.speed]
        raise Exception("Unknown mode: %s" % self.mode)


def stop():
    return Plan(consts.PLAN_STOP_MODE)
