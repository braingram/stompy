#!/usr/bin/env python
"""
Supply this a stance plan in body coordinates
it will produce plans in body coordinates
restriction will be updated with foot coordinates

it will produce 'requests' for plans that will be 'accepted'
"""

import numpy

from .. import consts
from .. import kinematics
from .. import signaler


class Foot(signaler.Signaler):
    r_max = 0.9
    r_thresh = 0.2
    dr_smooth = 0.5  # should be 0 -> 0.9999: higher = more dr smoothing

    radius = 42.0
    r_eps = numpy.log(0.1) / radius

    # TODO make these properties that cause plans to be re-sent
    stance_velocity = 4.0
    velocity_scale = 1.0

    step_size = 30.0
    swing_velocity = 8.0
    lower_velocity = -4.0
    lift_velocity = 4.0

    lower_height = -15.0
    lift_height = -5.0

    close_enough = 5.0

    def __init__(self, leg_number):
        super(Foot, self).__init__()
        self.leg_number = leg_number
        self.center = (66., 0.)
        self.state = None  # disabled
        self.leg_frame_target = None
        self.last_r = None
        self.last_time = None
        self.dr = 0.
        self.idr = 0.

    def set_state(self, state):
        self.state = state
        self.trigger('state', state)

    def set_target(self, tx, ty, frame):
        if frame == consts.PLAN_BODY_FRAME:
            # convert x, y to foot?
            tlx, tly, _ = kinematics.body.body_to_leg(
                self.leg_number, tx, ty, 0.)
            tbx, tby = tx, ty
        elif frame == consts.PLAN_LEG_FRAME:
            tlx, tly = tx, ty
            tbx, tby, _ = kinematics.body.leg_to_body(
                self.leg_number, tx, ty, 0.)
        else:
            raise ValueError("Invalid target frame: %s" % frame)
        # compute foot target (to tell when swing is done)
        self.leg_frame_target = (tlx, tly)
        self.swing_target = (
            tlx * self.step_size,
            tly * self.step_size)
        # compute plans (swing, lift, lower, stance)
        self.plans = {
            'halt': {'mode': consts.PLAN_STOP_MODE},
            'swing': {
                'mode': consts.PLAN_TARGET_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (
                    self.swing_target[0],
                    self.swing_target[1],
                    self.lift_height),
                'speed': self.swing_velocity,
            },
            'lift': {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (
                    -tlx * self.stance_velocity,
                    -tly * self.stance_velocity,
                    self.lift_velocity),
                'speed': 1.,
            },
            'lower': {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (
                    -tlx * self.stance_velocity,
                    -tly * self.stance_velocity,
                    self.lower_velocity),
                'speed': 1.,
            },
            'stance': {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (-tlx, -tly, 0.),
                'speed': self.stance_velocity,
            },
        }

    def calculate_restriction(self, x, y, z, t):
        cx, cy = self.center
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        self.r = numpy.exp(-self.r_eps * (d - self.radius))
        if self.last_r is not None:
            if (self.last_t >= t):
                raise ValueError(
                    "Time is incorrect: %s >= %s" % (self.last_t, t))
            self.idr = (self.r - self.last_r) / (t - self.last_t)
            self.dr = (
                self.dr * self.dr_smooth + self.idr * (1. - self.dr_smooth))
        self.last_r = self.r
        self.last_t = t

    def update(self, x, y, z, t):
        self.calculate_restriction(x, y, z, t)
        if self.state is None:
            return None
        ns = None
        if self.state == 'swing':
            # check against target position
            tx, ty = self.swing_target
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
            # continue moving until restricted and r is increasing
            if self.r > self.r_thresh and self.dr > 0:
                ns = 'lift'
        if self.r > self.r_max:
            ns = 'halt'
        # build request
        if ns is None:
            return None
        return {
            'state': ns,
            'plan': self.plans[ns],
        }


class Body(signaler.Signaler):
    def __init__(self):
        super(Body, self).__init__()
        self.max_feet_up = 1
        self.last_lift_times = {}
        self.feet = {}

    def process_request(self, leg_number, request):
        # is leg_number requesting halt?
        #   yes: stop all legs in stance
        # is leg_number asking to lift?
        #   yes: check n_feet up, check neighbors
        #   no: allow transition
        # is >1 leg restricted?
        #   yes: lift last recently moved
        pass
