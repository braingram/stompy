#!/usr/bin/env python

import time

import numpy


class Foot(object):
    r_max = 0.9
    r_thresh = 0.15
    center = (66.0, 0.0)
    radius = 42.0
    r_eps = numpy.log(0.1) / radius

    step_size = 20.0
    stance_velocity = 8.0
    swing_velocity = 16.0
    lower_velocity = 16.0
    lift_velocity = 16.0
    velocity_scale = 1.0

    lower_height = -15.0
    lift_height = -5.0

    close_enough = 5.0

    def __init__(self):
        self.state = 'stance'
        self.target = None
        self.last_update = time.time()

    def restriction(self, x, y, z):
        cx, cy = self.center
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.eps * (d - self.radius))

    def update(self, x, y, z, t):
        #dt = t - self.last_update
        r = self.restriction(x, y, z)
        ns = None
        if self.state == 'swing':
            # check against target position
            tx, ty = self.target
            d = ((tx - x) ** 2. + (ty - y) ** 2.) ** 0.5
            # TODO also check for increase in distance
            if d < self.close_enough:
                # if there, move to lower
                ns = 'lower'
        elif self.state == 'lower':
            # lower leg until z at lower_height
            if z <= self.lower_height:
                # if there, enter stance
                ns = 'stance'
        elif self.state == 'lift':
            # lift leg until z at lift_height
            if z >= self.lift_height:
                # if there, enter swing
                ns = 'swing'
        elif self.state == 'stance':
            # continue moving...
            pass
        self.last_update = t
        return r, ns
