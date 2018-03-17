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


neighbors = {
    1: [2, 6],
    2: [1, 3],
    3: [2, 4],
    4: [3, 5],
    5: [4, 6],
    6: [5, 1],
}


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
        self.halted = False
        self._do_halt = False
        self.state = None  # disabled
        self.leg_frame_target = None
        self.body_frame_target = None
        self.last_r = None
        self.last_time = None
        self.dr = 0.
        self.idr = 0.
        self.plans = {}
        self.halted_plans = {}

    def set_state(self, state):
        print("%s: %s[%s]" % (self.leg_number, state, self.state))
        self.state = state
        self.trigger('state', state)
        if state == 'halt':
            self.halted = True

    def halt(self):
        print("halt[%s,%s]" % (self.leg_number, self.state))
        self._do_halt = True
        self.halted = True

    def unhalt(self):
        self._do_halt = False
        self.halted = True

    def get_target(self):
        return (
            self.body_frame_target[0], self.body_frame_target[1],
            consts.PLAN_BODY_FRAME)

    def set_target(self, tx, ty, frame):
        if frame == consts.PLAN_BODY_FRAME:
            # convert x, y to foot?
            tlx, tly, _ = kinematics.body.body_to_leg_rotation(
                self.leg_number, tx, ty, 0.)
            tbx, tby = tx, ty
        elif frame == consts.PLAN_LEG_FRAME:
            tlx, tly = tx, ty
            tbx, tby, _ = kinematics.body.leg_to_body_rotation(
                self.leg_number, tx, ty, 0.)
        else:
            raise ValueError("Invalid target frame: %s" % frame)
        # compute foot target (to tell when swing is done)
        self.leg_frame_target = (tlx, tly)
        self.body_frame_target = (tbx, tby)
        self.swing_target = (
            self.center[0] + tlx * self.step_size,
            self.center[1] + tly * self.step_size)
        #print(
        #    self.leg_number, self.leg_frame_target, self.swing_target,
        #    tlx, tly, self.center)
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
        self.halted_plans = {
            'lift': {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (
                    0,
                    0,
                    self.lift_velocity),
                'speed': 1.,
            },
            'lower': {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': (
                    0,
                    0,
                    self.lower_velocity),
                'speed': 1.,
            },
            'swing': self.plans['swing'],
            'stance': self.plans['halt'],
            'halt': self.plans['halt'],
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
        if self._do_halt:
            print("update[%s], _do_halt" % self.leg_number)
        #if self.halted:
        #    print("update[%s], halted" % self.leg_number)
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
            if self.r > self.r_thresh and self.dr >= 0:
                ns = 'lift'
        elif self.state == 'halt':
            ns = 'lift'
        if self.r > self.r_max and not self.halted:
            ns = 'halt'
        # build request
        if self._do_halt:
            if ns is None:
                ns = self.state
            elif ns == 'lift':
                ns = 'halt'
            print("_do_halt %s: %s" % (self.leg_number, ns))
            self._do_halt = False
        if ns is None:
            return None
        if self.halted:
            plan = self.halted_plans[ns]
        else:
            plan = self.plans[ns]
        return {
            'state': ns,
            'plan': plan,
            'leg_number': self.leg_number,
        }


class Body(signaler.Signaler):
    def __init__(self, legs):
        """Takes leg controllers"""
        super(Body, self).__init__()
        self.max_feet_up = 1
        self.last_lift_times = {}
        self.legs = legs
        #self.halts = {}
        for i in self.legs:
            self.legs[i].on('request', lambda r, ln=i: self.on_request(r, ln))
            #self.halts[i] = False

    def on_request(self, request, leg_number):
        # is leg_number requesting halt?
        if request['state'] == 'halt':
            #self.halts[request['leg_number']] = True
            # accept halt
            request['accept']()
            # stop all legs in stance
            for i in self.legs:
                l = self.legs[i]
                if i != request['leg_number'] and not l.res.halted:
                    print("Halting: %s" % i)
                    l.res.halt()
                # TODO, fix lift, lower to straight up and down
                ## reset plan to 0, 0
                #l.res.set_target(0, 0, consts.PLAN_LEG_FRAME)
                #if i != request['leg_number']:
                #    l.send_plan(**l.res.plans[l.res.state])
            return
        # if all legs are out of halt, unhalt
        unhalt = False
        for i in self.legs:
            if self.legs[i].res.halted:
                unhalt = False
                break
        if unhalt:  # TODO don't do this every request
            for i in self.legs:
                self.legs[i].res.unhalt()
        # is leg_number asking to lift?
        if request['state'] == 'lift':
            # check n_feet up
            states = {
                self.legs[i].leg_number: self.legs[i].res.state
                for i in self.legs}
            n_up = len([
                s for s in states.values() if s not in ('stance', 'halt')])
            # check if neighbors are up
            ns = neighbors[request['leg_number']]
            n_states = [self.legs[n].res.state for n in ns]
            ns_up = len([s for s in n_states if s not in ('stance', 'halt')])
            if ns_up == 0 and n_up < self.max_feet_up:
                request['accept']()
            # else don't allow the request
            return
        request['accept']()
        # TODO is >1 leg restricted?
        #   [how to do this with current setup?]
        #   [maybe wait to get a few feet worth of data]
        #   yes: lift last recently moved
        # TODO scale speed by max restriction
