#!/usr/bin/env python

import time

import numpy

from .. import consts
from .. import signaler
from .. import transforms


class Odometer(signaler.Signaler):
    def __init__(self):
        super(Odometer, self).__init__()
        self.reset()

    def reset(self):
        self.target = None
        self.last_update = None
        self.last_update = None
        self._rticks = 0.
        self.position = [0., 0., 0.]
        self.angle = 0.
        self.poses = []

    def get_path(self, simplify=True, zero=True, max_n=1200):
        # simplify path to only 1 inch apart points
        if len(self.poses) == 0:
            return []

        # return poses from last to first
        pts = [self.poses[-1], ]

        # if zero
        if zero:
            p = pts[0]['position']
            a = pts[0]['angle']
            dz = p[2]
            #T = transforms.affine_2d(-p[0], -p[1], -a)
            T = (
                transforms.rotation_2d(-a) *
                transforms.translation_2d(-p[0], -p[1]))

            def transform_point(pt, T=T, dz=dz, a=a):
                x, y = transforms.transform_2d(
                    T,
                    pt['position'][0],
                    pt['position'][1])
                return {
                    'position': [x, y, pt['position'][2] - dz],
                    'angle': pt['angle'] - a,
                    'timestamp': pt['timestamp'],
                }

            pts[0] = transform_point(pts[0])
        else:
            transform_point = lambda pt: pt

        for pt in self.poses[:-1][::-1]:
            if len(pts) >= max_n:
                break
            pt = transform_point(pt)
            if not simplify:
                pts.append(pt)
                continue
            # check if pt was > 1 inch from pts[-1]
            pp = pts[-1]['position']
            p = pt['position']
            d = numpy.linalg.norm((
                p[0] - pp[0], p[1] - pp[1], p[2] - pp[2]))
            if d >= 1.0:
                pts.append(pt)
        return pts

    def set_target(self, target):
        if target != self.target:
            self._rticks = 0.
            self.target = target  # rotation_center, speed, dz

    def update(self, target=None, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        if self.last_update is None or self.target is None:
            self.last_update = timestamp
            if target is not None:
                self.set_target(target)
            return

        t = timestamp
        dt = t - self.last_update
        self.last_update = t

        # find next position and angle

        # follow current target plan for dt seconds
        ticks = dt / consts.PLAN_TICK + self._rticks
        iticks = int(numpy.floor(ticks))
        # store residual partial ticks to be reused if the target is the same
        self._rticks = ticks - iticks

        T = transforms.rotation_about_point_2d(
            self.target.rotation_center[0],
            self.target.rotation_center[1],
            self.target.speed)
        pos = [0., 0.]
        for _ in range(iticks):
            pos = transforms.transform_2d(T, *pos)
        sa = numpy.sin(self.angle)
        ca = numpy.cos(self.angle)
        self.position[0] -= ca * pos[0] - sa * pos[1]
        self.position[1] -= sa * pos[0] + ca * pos[1]
        self.position[2] -= self.target.dz * iticks
        self.angle -= iticks * self.target.speed

        self.poses.append({
            'angle': self.angle,
            'position': self.position[:],
            'timestamp': timestamp,
        })
        self.trigger('pose', self.poses[-1])

        # then set new plan
        if target is not None:
            self.set_target(target)
