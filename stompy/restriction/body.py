#!/usr/bin/env python
"""
Supply this a stance plan in body coordinates
it will produce plans in body coordinates
restriction will be updated with foot coordinates

it will produce 'requests' for plans that will be 'accepted'
"""

import numpy

from . import cfg
from .. import consts
from .. import kinematics
from . import leg
from .. import signaler


class BodyTarget(object):
    def __init__(self, rotation_center, speed, dz):
        self.rotation_center = rotation_center
        self.speed = speed
        self.dz = dz


class Body(signaler.Signaler):
    def __init__(self, legs, **kwargs):
        """Takes leg controllers"""
        super(Body, self).__init__()
        self.cfg = cfg.RestrictionConfig()
        #self.r_thresh = 0.2
        #self.r_max = 0.9
        self.legs = legs
        self.cfg.max_feet_up = 1
        #if len(self.legs) > 5:
        #    #self.max_feet_up = 3
        #    self.cfg.max_feet_up = 3
        #else:
        #    #self.max_feet_up = 1
        #    self.cfg.max_feet_up = 1
        self.feet = {}
        self.halted = False
        self.enabled = False
        self.target = None
        inds = sorted(self.legs)
        self.neighbors = {}
        if len(inds) > 1:
            for (i, n) in enumerate(inds):
                if i == 0:
                    self.neighbors[n] = [
                        inds[len(inds) - 1], inds[i+1]]
                elif i == len(inds) - 1:
                    self.neighbors[n] = [inds[i - 1], inds[0]]
                else:
                    self.neighbors[n] = [inds[i - 1], inds[i + 1]]
        for i in self.legs:
            self.feet[i] = leg.Foot(self.legs[i], self.cfg, **kwargs)
            #self.feet[i].on('state', lambda s, ln=i: self.on_state(s, ln))
            self.feet[i].on(
                'restriction', lambda s, ln=i: self.on_restriction(s, ln))
        self.disable()

    def enable(self, foot_states):
        self.enabled = True
        self.halted = False
        # TODO set foot states, target?

    def set_speed(self, speed_scalar):
        self.cfg.speed_scalar = speed_scalar
        self.set_target(self.target)

    def calc_stance_speed(self, bxy, mag):
        # scale to pid future time ms
        speed = mag * self.cfg.get_speed('stance') * consts.PLAN_TICK
        # find furthest foot
        #x, y, z = (numpy.abs(radius), 0., 0.)
        x, y = bxy
        z = 0.
        #x, y, z = (numpy.abs(radius), 0., 0.)
        mr = None
        for i in self.feet:
            tx, ty, tz = kinematics.body.body_to_leg(i, x, y, z)
            r = tx * tx + ty * ty + tz * tz
            if mr is None or r > mr:
                mr = r
        mr = numpy.sqrt(mr)
        # TODO account for radius sign
        rspeed = speed / mr
        if numpy.abs(rspeed) > self.cfg.get_speed('angular'):
            print("Limiting because of angular speed")
            rspeed = self.cfg.get_speed('angular') * numpy.sign(rspeed)
        #print(mr, speed, rspeed)
        if self.cfg.speed_by_restriction:
            rs = self.get_speed_by_restriction()
        else:
            rs = 1.
        return rspeed * rs

    def set_target(self, target, update_swing=True):
        if not isinstance(target, BodyTarget):
            raise ValueError("Body.set_target requires BodyTarget")
        if self.halted:
            # set new pre_halt target
            self._pre_halt_target = target
            # set stance target to stop
            target = BodyTarget((0., 0.), 0., 0.)
            # only update non-swing
            update_swing = False
        self.target = target
        for i in self.feet:
            self.feet[i].set_target(
                target, update_swing=update_swing)

    def disable(self):
        self.enabled = False
        for i in self.feet:
            self.feet[i].set_state(None)

    #def on_state(self, state, leg_number):
    #    pass

    def halt(self):
        if not self.halted:
            print("HALT")
            self._pre_halt_target = self.target
            self.set_target(BodyTarget((0., 0.), 0., 0.), update_swing=False)
            self.halted = True

    def get_speed_by_restriction(self):
        rmax = max([
            self.feet[i].restriction['r'] for i in self.feet
            if self.feet[i].state not in ('swing', 'lower')])
        return max(0., min(1., 1. - rmax))

    def on_restriction(self, restriction, leg_number):
        if not self.enabled:
            return
        if self.halted and restriction['r'] < self.cfg.r_max:
            # unhalt?
            maxed = False
            for i in self.feet:
                # make sure foot is not in swing (or lower?)
                if self.feet[i].state in ('swing', 'lower'):
                    continue
                if self.feet[i].restriction['r'] > self.cfg.r_max:
                    maxed = True
            if not maxed:
                print("Unhalt")
                self.halted = False
                self.set_target(self._pre_halt_target, update_swing=False)
            return
        if restriction['r'] > self.cfg.r_max and not self.halted:
            self.halt()
            return
        # TODO scale stance speed by restriction?
        if (
                restriction['r'] > self.cfg.r_thresh and
                self.feet[leg_number].state == 'stance'):
            # lift?
            # check n_feet up
            states = {i: self.feet[i].state for i in self.feet}
            n_up = len([
                s for s in states.values() if s not in ('stance', 'wait')])
            # check if neighbors are up
            if len(self.neighbors.get(leg_number, [])) == 0:
                return
            ns = self.neighbors[leg_number]
            n_states = [states[n] for n in ns]
            ns_up = len([s for s in n_states if s not in ('stance', 'wait')])
            # check if any other feet are restricted:
            last_lift_times = {}
            for ln in self.feet:
                if ln == leg_number:
                    last_lift_times[ln] = self.feet[ln].last_lift_time
                    continue
                if states[ln] not in ('stance', 'wait'):
                    continue
                if self.feet[ln].restriction['r'] > self.cfg.r_thresh:
                    # found another restricted foot
                    #other_restricted.append(ln)
                    last_lift_times[ln] = self.feet[ln].last_lift_time
            #  yes? pick least recently lifted
            if ns_up == 0 and n_up < self.cfg.max_feet_up:
                n_can_lift = self.cfg.max_feet_up - n_up
                if len(last_lift_times) > n_can_lift:
                    # TODO prefer lifting of feet with
                    # restriction_modifier != 0
                    # only allow this foot if it was moved later than
                    # the other restricted feet
                    ln_by_lt = sorted(
                        last_lift_times, key=lambda ln: last_lift_times[ln])
                    if leg_number in ln_by_lt[:n_can_lift+1]:
                        self.feet[leg_number].set_state('lift')
                else:
                    self.feet[leg_number].set_state('lift')
