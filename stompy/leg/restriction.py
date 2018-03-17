#!/usr/bin/env python
"""
Supply this a stance plan in body coordinates
it will produce plans in body coordinates
restriction will be updated with foot coordinates

it will produce 'requests' for plans that will be 'accepted'
"""

import time

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
    stance_velocity = 3.  # TODO make these configurable and body wide
    lift_velocity = 4.
    lower_velocity = -4.
    swing_velocity = 8.
    step_size = 30.

    def __init__(
            self, leg, radius=42, eps=0.1, center=(66., 0.), dr_smooth=0.5,
            close_enough=5., lift_height=-5.0, lower_height=-15.0):
        super(Foot, self).__init__()
        self.leg = leg
        self.leg.on('xyz', self.on_xyz)
        self.leg.on('angles', self.on_angles)
        self.radius = radius
        self.eps = eps
        self.r_eps = numpy.log(eps) / self.radius
        self.center = center
        self.last_lift_time = time.time()
        self.leg_target = None
        self.body_target = None
        self.swing_target = None
        self.lift_height = lift_height
        self.lower_height = lower_height
        self.close_enough = close_enough
        # stance -> lift -> swing -> lower -> wait
        self.state = None
        self.restriction = None
        self.dr_smooth = dr_smooth
        self.xyz = None
        self.angles = None

    def send_plan(self):
        print("res.send_plan: %s" % self.state)
        if self.state is None:
            # TODO always stop on disable?
            self.leg.send_plan(mode=consts.PLAN_STOP_MODE)
        elif self.state in ('stance', 'wait'):
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(-self.leg_target[0], -self.leg_target[1], 0.),
                speed=self.stance_velocity)
        elif self.state == 'lift':
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    -self.leg_target[0] * self.stance_velocity,
                    -self.leg_target[1] * self.stance_velocity,
                    self.lift_velocity),
                speed=1.)
        elif self.state == 'swing':
            self.leg.send_plan(
                mode=consts.PLAN_TARGET_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    self.swing_target[0],
                    self.swing_target[1],
                    self.lift_height),
                speed=self.swing_velocity)
        elif self.state == 'lower':
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    -self.leg_target[0] * self.stance_velocity,
                    -self.leg_target[1] * self.stance_velocity,
                    self.lower_velocity),
                speed=1.)

    def set_target(self, xy, update_swing=True):
        self.body_target = xy
        lx, ly, _ = kinematics.body.body_to_leg_rotation(
            self.leg.leg_number, xy[0], xy[1], 0.)
        self.leg_target = (lx, ly)
        if update_swing:
            self.swing_target = (
                self.center[0] + lx * self.step_size,
                self.center[1] + ly * self.step_size)
        self.send_plan()

    def set_state(self, state):
        self.state = state
        if self.state == 'lift':
            self.last_lift_time = time.time()
        self.send_plan()
        self.trigger('state', state)

    def calculate_restriction(self, xyz):
        cx, cy = self.center
        d = ((cx - xyz['x']) ** 2. + (cy - xyz['y']) ** 2.) ** 0.5
        r = numpy.exp(-self.r_eps * (d - self.radius))
        if self.restriction is not None:
            pt = self.restriction['time']
            dt = (xyz['time'] - pt)
            idr = (r - self.restriction['r']) / dt
            dr = (
                self.restriction['dr'] * self.dr_smooth +
                idr * (1. - self.dr_smooth))
        else:  # first update
            idr = 0.
            dr = 0.
        self.restriction = {
            'time': xyz['time'], 'r': r, 'dr': dr, 'idr': idr}
        self.trigger('restriction', self.restriction)

    def _is_swing_done(self, xyz):
        tx, ty = self.swing_target
        d = ((tx - xyz['x']) ** 2. + (ty - xyz['y']) ** 2.) ** 0.5
        # TODO also check for increase in distance
        return d < self.close_enough

    def on_xyz(self, xyz):
        self.xyz = xyz
        if self.angles is not None:
            self.update()

    def on_angles(self, angles):
        self.angles = angles
        if self.xyz is not None:
            self.update()

    def update(self):
        # TODO if angles['valid'] is False?
        self.calculate_restriction(self.xyz)
        new_state = None
        if self.state is None:  # restriction control is disabled
            self.xyz = None
            self.angles = None
            return
        elif self.state == 'swing':
            if self._is_swing_done(self.xyz):
                new_state = 'lower'
        elif self.state == 'lower':
            # TODO check for loaded >L lbs
            if self.xyz['z'] < self.lower_height:
                new_state = 'wait'
        elif self.state == 'wait':
            if self.restriction['dr'] > 0:
                new_state = 'stance'
        #elif self.state == 'stance'
        elif self.state == 'lift':
            # TODO check for unloaded and >Z inches off ground
            if self.xyz['z'] > self.lift_height:
                new_state = 'swing'
        # clear xyz and angles cache
        self.xyz = None
        self.angles = None
        if new_state is not None:
            self.set_state(new_state)


class Body(signaler.Signaler):
    def __init__(self, legs, **kwargs):
        """Takes leg controllers"""
        # TODO enable/disable
        super(Body, self).__init__()
        self.r_thresh = 0.2
        self.r_max = 0.9
        self.max_feet_up = 3
        self.legs = legs
        self.feet = {}
        self.halted = False
        self.enabled = False
        self.target = None
        for i in self.legs:
            self.feet[i] = Foot(self.legs[i], **kwargs)
            #self.feet[i].on('state', lambda s, ln=i: self.on_state(s, ln))
            self.feet[i].on(
                'restriction', lambda s, ln=i: self.on_restriction(s, ln))

    def enable(self, foot_states):
        self.enabled = True
        # TODO set foot states, target?

    def set_target(self, xy, update_swing=True):
        self.target = xy
        for i in self.feet:
            self.feet[i].set_target(xy, update_swing=update_swing)

    def disable(self):
        self.enabled = False
        for i in self.feet:
            self.feet[i].set_state(None)

    #def on_state(self, state, leg_number):
    #    pass

    def on_restriction(self, restriction, leg_number):
        if not self.enabled:
            return
        if self.halted and restriction['r'] < self.r_max:
            # unhalt?
            maxed = False
            for i in self.feet:
                if self.feet[i].restriction['r'] > self.r_max:
                    maxed = True
            if not maxed:
                print("Unhalt")
                self.halted = False
                self.set_target(self._pre_halt_target, update_swing=False)
        if restriction['r'] > self.r_max and not self.halted:
            # halt!
            self._pre_halt_target = self.target
            self.set_target((0., 0.), update_swing=False)
            self.halted = True
            print("Halt")
            return
        # TODO scale stance speed by restriction?
        if (
                restriction['r'] > self.r_thresh and
                self.feet[leg_number].state == 'stance'):
            # lift?
            # check n_feet up
            states = {i: self.feet[i].state for i in self.feet}
            n_up = len([
                s for s in states.values() if s not in ('stance', 'wait')])
            # check if neighbors are up
            ns = neighbors[leg_number]
            n_states = [states[n] for n in ns]
            ns_up = len([s for s in n_states if s not in ('stance', 'wait')])
            # TODO check if any other feet are restricted:
            #  yes? pick least recently lifted
            if ns_up == 0 and n_up < self.max_feet_up:
                self.feet[leg_number].set_state('lift')
