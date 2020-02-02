#!/usr/bin/env python

import math
import os
import time

import numpy
import pylab

from .. import consts
from .. import kinematics
from .. import signaler
from .. import transforms


class Odometer(signaler.Signaler):
    def __init__(self):
        super(Odometer, self).__init__()
        self.enabled = False
        self.pose_update_distance = 1.
        self.max_pose_points = 100
        self.reset()

    def reset(self):
        self.target = None
        self.last_update = None
        self.last_update = None
        self._rticks = 0.
        self.position = [0., 0., 0.]
        self.angle = 0.
        self.timestamp = time.time()
        self.poses = []

    def get_path(self):
        pose = {
            'position': self.position[:],
            'angle': self.angle,
            'timestamp': self.timestamp,
        }

        # simplify path to only 1 inch apart points
        if len(self.poses) == 0:
            return [pose, ]

        # return poses from last to first
        p = self.position
        a = self.angle
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

        pts = [transform_point(pose), ]
        pts.extend(transform_point(pt) for pt in self.poses[::-1])

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

        if not self.enabled:
            return

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
        sa = math.sin(self.angle)
        ca = math.cos(self.angle)
        self.position[0] -= ca * pos[0] - sa * pos[1]
        self.position[1] -= sa * pos[0] + ca * pos[1]
        self.position[2] -= self.target.dz * iticks
        self.angle -= iticks * self.target.speed
        self.timestamp = timestamp
        self.pose = {
            'angle': self.angle,
            'position': self.position[:],
            'timestamp': self.timestamp,
        }
        if len(self.poses) == 0:
            self.poses.append(self.pose)
        else:
            # check that pose is > pose_distance
            pp = self.poses[-1]['position']
            p = self.pose['position']
            d = numpy.linalg.norm((
                p[0] - pp[0], p[1] - pp[1], p[2] - pp[2]))
            if d >= self.pose_update_distance:
                self.poses.append(self.pose)
            if len(self.poses) > self.max_pose_points:
                self.poses = self.poses[-self.max_pose_points:]
        self.trigger('pose', self.pose)

        # then set new plan
        if target is not None:
            self.set_target(target)



class FakeTerrain(signaler.Signaler):
    """
    Load image that is terrain
    Take in pose (position and angle)
    Compute foot positions (as pixel coordinates)
    Update fake feet lower heights
    """
    def __init__(self, fn=None):
        if fn is None:
            fn = '~/.stompy/terrain.png'
        self.load_image(fn)

    def load_image(self, fn, hf=None):
        fn = os.path.realpath(os.path.expanduser(fn))
        if not os.path.exists(fn):
            self.im = None
            return
        self.im = pylab.imread(fn)
        if self.im.ndim == 3:
            self.im = numpy.mean(self.im, axis=2)
        self._im_center = (
            self.im.shape[0] / 2,
            self.im.shape[0] / 2,
        )
        if hf is None:
            #hf = lambda v: -(40 + 20. * v)
            hf = lambda v: -(20 + 60. * v)
        self.hf = hf

    def new_pose(self, pose, legs):
        if self.im is None:
            return
        # compute foot positions in pixel coordinates from pose
        T = (
            transforms.rotation_2d(pose['angle']) *
            transforms.translation_2d(
                pose['position'][0], pose['position'][1]))

        # update legs with heights computed from pixels
        for ln in legs:
            leg = legs[ln]
            if not hasattr(leg, '_loaded_height'):
                # only works for fake legs
                continue
            # convert to body coodrinates
            bxyz = kinematics.body.leg_to_body(
                ln, leg.xyz['x'], leg.xyz['y'], leg.xyz['z'])
            # convert to 'global' coordinates
            x, y = transforms.transform_2d(T, bxyz[0], bxyz[1])
            # offset by pixel center
            ix = min(
                self.im.shape[1] - 1, max(
                    0, int(numpy.round(x + self._im_center[0]))))
            iy = min(
                self.im.shape[0] - 1, max(
                    0, int(numpy.round(y + self._im_center[1]))))
            v = self.im[iy, ix]
            # update height at which leg becomes loaded including pose z
            leg._loaded_height = self.hf(v) - pose['position'][2]
            #if ln == 1:
            #    print('terrain:', ix, iy, v, leg._loaded_height)
        # TODO compute local body slope (pitch, roll)
        ix = min(
            self.im.shape[1] - 1, max(
                0, int(numpy.round(pose['position'][0]))))
        iy = min(
            self.im.shape[0] - 1, max(
                0, int(numpy.round(pose['position'][1]))))
        #print("Pose:", ix, iy)
