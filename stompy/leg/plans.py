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
from .. import transforms


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


def follow_plan(xyz, plan, dt=None):
    if dt is None:
        dt = consts.PLAN_TICK
    # TODO xyz as dict or as tuple
    xyz = list(xyz)[:]
    # TODO handle estop OUTSIDE this function
    if plan is None:
        return xyz
    if plan.mode == consts.PLAN_STOP_MODE:
        return xyz
    if plan.frame != consts.PLAN_LEG_FRAME:
        # only works for leg frame for right now
        raise Exception
    if plan.mode == consts.PLAN_VELOCITY_MODE:
        lx, ly, lz = plan.linear
        xyz[0] += lx * dt * plan.speed
        xyz[1] += ly * dt * plan.speed
        xyz[2] += lz * dt * plan.speed
    elif plan.mode == consts.PLAN_TARGET_MODE:
        tx, ty, tz = plan.linear
        lx = tx - xyz[0]
        ly = ty - xyz[1]
        lz = tz - xyz[2]
        l = ((lx * lx) + (ly * ly) + (lz * lz)) ** 0.5
        if l < (plan.speed * dt) or l < 0.01:
            xyz[0] = tx
            xyz[1] = ty
            xyz[2] = tz
        else:
            lx /= l
            ly /= l
            lz /= l
            xyz[0] += lx * dt * plan.speed
            xyz[1] += ly * dt * plan.speed
            xyz[2] += lz * dt * plan.speed
    elif plan.mode == consts.PLAN_ARC_MODE:
        lx, ly, lz = plan.linear
        ax, ay, az = plan.angular
        #ax *= plan.speed * consts.PLAN_TICK
        #ay *= plan.speed * consts.PLAN_TICK
        #az *= plan.speed * consts.PLAN_TICK
        ax *= plan.speed * dt
        ay *= plan.speed * dt
        az *= plan.speed * dt
        T = transforms.rotation_about_point_3d(
            lx, ly, lz, ax, ay, az, degrees=False)
        nx, ny, nz = transforms.transform_3d(
            T, xyz[0], xyz[1], xyz[2])

        #self._ddt += dt
        #nx, ny, nz = xyz[0], xyz[1], xyz[2]
        #while self._ddt >= consts.PLAN_TICK:
        #    #nx, ny, nz = transforms.transform_3d(
        #    #    plan.matrix, nx, ny, nz)
        #    nx, ny, nz = transforms.transform_3d(T, nx, ny, nz)
        #    self._ddt -= consts.PLAN_TICK

        xyz[0] = nx
        xyz[1] = ny
        xyz[2] = nz
    elif plan.mode == consts.PLAN_MATRIX_MODE:
        # call many times if dt > 4 ms)
        #print("_follow_plan:", plan.matrix)
        nx, ny, nz = xyz[0], xyz[1], xyz[2]
        while dt > 0:
            nx, ny, nz = transforms.transform_3d(
                plan.matrix, nx, ny, nz)
            dt -= consts.PLAN_TICK
        xyz[0] = nx
        xyz[1] = ny
        xyz[2] = nz
    return xyz
