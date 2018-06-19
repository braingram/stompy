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
from .. import geometry
from .. import kinematics
from .. import signaler
from .. import transforms


max_radius = 100000.


class RestrictionConfig(signaler.Signaler):
    def __init__(self):
        super(RestrictionConfig, self).__init__()
        self.speeds = {
            'stance': 6.,
            'lift': 6.,
            'lower': 6.,
            'swing': 12.,
            'angular': 0.0005}
        self.step_size = 25.
        self.r_thresh = 0.2
        self.r_max = 0.9
        self.max_feet_up = 0
        self.speed_scalar = 1.
        self.height_slop = 5.
        self.foot_center = (55., 0.)
        self.dr_smooth = 0.5
        self.eps = 0.9
        self.lift_height = 8.0
        self.lower_height = -50.0
        self.unloaded_weight = 600.
        self.loaded_weight = 300.
        self.swing_slop = 5.0

    def get_speed(self, mode):
        if mode not in self.speeds:
            raise ValueError("Invalid restriction speed mode: %s" % mode)
        return self.speeds[mode] * self.speed_scalar


class Foot(signaler.Signaler):
    def __init__(
            self, leg, cfg,
            #radius=20, eps=0.9, center=(55., 0.), dr_smooth=0.5,
            #eps=0.9, center=(55., 0.), dr_smooth=0.5,
            #close_enough=5., lift_height=8.0, lower_height=-50.0,
            #height_slop=5.,
            #unloaded_weight=600, loaded_weight=300):
            ):
        super(Foot, self).__init__()
        self.leg = leg
        self.cfg = cfg
        self.limits = geometry.get_limits(self.leg.leg_number)
        self.leg.on('xyz', self.on_xyz)
        self.leg.on('angles', self.on_angles)
        #self.radius = radius
        #self.eps = eps
        #self.r_eps = numpy.log(eps) / self.radius
        #self.center = center
        self.last_lift_time = time.time()
        self.leg_target = None
        self.body_target = None
        self.swing_target = None
        #self.lift_height = lift_height
        self.unloaded_height = None
        #self.height_slop = height_slop
        #self.unloaded_weight = unloaded_weight
        #self.loaded_weight = loaded_weight
        #self.lower_height = lower_height
        #self.close_enough = close_enough
        # stance -> lift -> swing -> lower -> wait
        self.state = None
        self.restriction = None
        #self.dr_smooth = dr_smooth
        self.xyz = None
        self.angles = None
        self.restriction_modifier = 0.

    def send_plan(self):
        print("res.send_plan: [%s]%s" % (self.leg.leg_number, self.state))
        if self.state is None or self.leg_target is None:
            # TODO always stop on disable?
            self.leg.send_plan(mode=consts.PLAN_STOP_MODE)
        elif self.state in ('stance', 'wait'):
            #self.leg.send_plan(
            #    mode=consts.PLAN_VELOCITY_MODE,
            #    frame=consts.PLAN_LEG_FRAME,
            #    linear=(-self.leg_target[0], -self.leg_target[1], 0.),
            #    speed=self.cfg.get_speed('stance'))
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                #matrix=numpy.matrix(numpy.identity(4)),
                matrix=self.leg_target,
                speed=0)
        elif self.state == 'lift':
            v = self.cfg.get_speed('lift')
            T = self.leg_target * transforms.translation_3d(0, 0, v * 0.004)
            #T = self.leg_target
            #print(self.leg_target, T)
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=T,
                speed=0)
            #v = self.cfg.get_speed('stance')
            #self.leg.send_plan(
            #    mode=consts.PLAN_VELOCITY_MODE,
            #    frame=consts.PLAN_LEG_FRAME,
            #    linear=(
            #        -self.leg_target[0] * v,
            #        -self.leg_target[1] * v,
            #       self.cfg.get_speed('lift')),
            #    speed=1.)
        elif self.state == 'swing':
            z = self.unloaded_height + self.cfg.lift_height
            self.leg.send_plan(
                mode=consts.PLAN_TARGET_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    self.swing_target[0],
                    self.swing_target[1],
                    z),
                speed=self.cfg.get_speed('swing'))
        elif self.state == 'lower':
            v = -self.cfg.get_speed('lower')
            T = self.leg_target * transforms.translation_3d(0, 0, v * 0.004)
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=T,
                speed=0)
            #v = self.cfg.get_speed('stance')
            #self.leg.send_plan(
            #    mode=consts.PLAN_VELOCITY_MODE,
            #    frame=consts.PLAN_LEG_FRAME,
            #    linear=(
            #        -self.leg_target[0] * v,
            #        -self.leg_target[1] * v,
            #        -self.cfg.get_speed('lower')),
            #    speed=1.)

    def set_target(self, R, xyz, update_swing=True):
        # compute swing target
        # rx, ly, az
        # convert rx, ly to rotation about point
        # convert az to velocity (z)
        # combine rotation and velocity
        # R = (radius, speed)
        #lR = kinematics.body.body_to_leg_matrix(self.leg.leg_number, R)
        rx, ry, rz = kinematics.body.body_to_leg(
            self.leg.leg_number, R[0], 0, 0)
        lR = transforms.rotation_about_point_3d(
            rx, ry, rz, 0, 0, R[1])
        self.leg_target = lR
        #self.body_target = xyz
        #lx, ly, _ = kinematics.body.body_to_leg_rotation(
        #    self.leg.leg_number, xyz[0], xyz[1], 0.)
        #self.leg_target = (lx, ly)
        if update_swing:
            # TODO optimized swing target
            self.swing_target = (
                self.cfg.foot_center[0], self.cfg.foot_center[1])
            #self.swing_target = (
            #    self.center[0] + lx * self.cfg.step_size,
            #    self.center[1] + ly * self.cfg.step_size)
        self.send_plan()

    def set_state(self, state):
        self.state = state
        if self.state == 'lift':
            self.unloaded_height = None
            self.last_lift_time = time.time()
        self.send_plan()
        self.trigger('state', state)

    def calculate_restriction(self, xyz, angles):
        # use angle limits
        r = 0
        for j in ('hip', 'thigh', 'knee'):
            jmin, jmax = self.limits[j]
            jmid = (jmax + jmin) / 2.
            if angles[j] > jmid:
                jl = angles[j] - jmax
            else:
                jl = jmin - angles[j]
            r = max(min(1.0, numpy.exp(self.cfg.eps * jl)), r)
            #if self.leg.leg_number == 1:
            #    print(j, jl, r)
        # TODO use calf angle
        #cx, cy = self.center
        #d = ((cx - xyz['x']) ** 2. + (cy - xyz['y']) ** 2.) ** 0.5
        #r = numpy.exp(-self.r_eps * (d - self.radius))
        # TODO restriction modifier
        r += self.restriction_modifier
        if self.restriction is not None:
            pt = self.restriction['time']
            dt = (xyz['time'] - pt)
            idr = (r - self.restriction['r']) / dt
            dr = (
                self.restriction['dr'] * self.cfg.dr_smooth +
                idr * (1. - self.cfg.dr_smooth))
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
        return d < self.cfg.swing_slop

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
        self.calculate_restriction(self.xyz, self.angles)
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
            #if self.xyz['z'] < self.lower_height:
            if (
                    (
                        abs(self.xyz['z'] - self.cfg.lower_height) <
                        self.cfg.height_slop) and
                    self.angles['calf'] > self.cfg.loaded_weight):
                new_state = 'wait'
        elif self.state == 'wait':
            if self.restriction['dr'] > 0:
                new_state = 'stance'
        #elif self.state == 'stance'
        elif self.state == 'lift':
            # check for unloaded and >Z inches off ground
            if (
                    self.unloaded_height is None and
                    self.angles['calf'] < self.cfg.unloaded_weight):
                self.unloaded_height = self.xyz['z']
            if (
                    self.unloaded_height is not None and
                    self.xyz['z'] > (self.unloaded_height + self.cfg.lift_height)):
                new_state = 'swing'
            #if self.xyz['z'] > self.lift_height:
            #    new_state = 'swing'
        # clear xyz and angles cache
        self.xyz = None
        self.angles = None
        if new_state is not None:
            print("setting new state: %s" % new_state)
            self.set_state(new_state)


class Body(signaler.Signaler):
    def __init__(self, legs, **kwargs):
        """Takes leg controllers"""
        super(Body, self).__init__()
        self.cfg = RestrictionConfig()
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
            self.feet[i] = Foot(self.legs[i], self.cfg, **kwargs)
            #self.feet[i].on('state', lambda s, ln=i: self.on_state(s, ln))
            self.feet[i].on(
                'restriction', lambda s, ln=i: self.on_restriction(s, ln))
        self.disable()

    def enable(self, foot_states):
        self.enabled = True
        # TODO set foot states, target?

    def set_speed(self, speed_scalar):
        self.cfg.speed_scalar = speed_scalar
        self.set_target(self.target)

    def set_target(self, xyz, update_swing=True):
        self.target = xyz
        # convert target x (rx) y (ry) to rotation about point
        radius_axis = xyz[0]
        if numpy.abs(radius_axis) < 0.001:  # sign(0.0) == 0.
            radius = max_radius
        else:
            radius = (
                numpy.sign(radius_axis) * max_radius /
                2. ** (numpy.log2(max_radius) * numpy.abs(radius_axis)))
        # TODO scale to pid future time ms
        speed = xyz[1] * self.cfg.get_speed('stance') * 0.004
        if abs(radius) < 0.1:
            rspeed = 0.
        else:
            # find furthest foot
            x, y, z = (numpy.abs(radius), 0., 0.)
            mr = None
            for i in self.feet:
                tx, ty, tz = kinematics.body.body_to_leg(i, x, y, z)
                r = tx * tx + ty * ty + tz * tz
                if mr is None or r > mr:
                    mr = r
            mr = numpy.sqrt(mr)
            rspeed = speed / mr * numpy.sign(radius)
            if numpy.abs(rspeed) > self.cfg.get_speed('angular'):
                rspeed = self.cfg.get_speed('angular') * numpy.sign(rspeed)
            print(mr, speed, rspeed)
        R = (radius, rspeed)
        #R = transforms.rotation_about_point_3d(
        #    radius, 0, 0, 0, 0, rspeed)
        print("Body matrix:", R)
        # TODO add up/down T
        for i in self.feet:
            self.feet[i].set_target(R, xyz, update_swing=update_swing)

    def disable(self):
        self.enabled = False
        for i in self.feet:
            self.feet[i].set_state(None)

    #def on_state(self, state, leg_number):
    #    pass

    def halt(self):
        if not self.halted:
            self._pre_halt_target = self.target
            self.set_target((0., 0., 0.), update_swing=False)
            self.halted = True

    def on_restriction(self, restriction, leg_number):
        if not self.enabled:
            return
        if self.halted and restriction['r'] < self.cfg.r_max:
            # unhalt?
            maxed = False
            for i in self.feet:
                if self.feet[i].restriction['r'] > self.cfg.r_max:
                    maxed = True
            if not maxed:
                print("Unhalt")
                self.halted = False
                self.set_target(self._pre_halt_target, update_swing=False)
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
            # TODO check if any other feet are restricted:
            #  yes? pick least recently lifted
            if ns_up == 0 and n_up < self.cfg.max_feet_up:
                self.feet[leg_number].set_state('lift')
