#!/usr/bin/env python

from .. import transforms


class Gait(object):
    def __init__(self):
        self.stance = None
        self.lower = None
        self.lift = None
        self.swing = None
        self.lift_speed = 0.01
        self.lower_speed = 0.01

    def recompute(self, cx, cy, angle, degrees=False):
        self.stance = transforms.rotation_about_point_3d(
            cx, cy, 0., 0., 0., angle, degrees=degrees)
        self.lower = (
            transforms.translation_3d(0., 0., self.lower_speed) * self.stance)
        self.lift = (
            transforms.translation_3d(0., 0., -self.lift_speed) * self.stance)
        # TODO swing to ideal point
        self.swing = (0., 0., 0.)
