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


def swing_position_from_intersections(tc, rspeed, c0, ipts, step_ratio):
    # if len(ipts) == 0, return to center?
    if len(ipts) == 0:
        return c0
    # if rspeed > 0: rotating clockwise, find point counterclockwise to 0
    # if rspeed < 0: rotating counterclockwise, find point clockwise to 0
    tc = numpy.array(tc)
    cv = numpy.array(c0) - tc
    cvn = cv / numpy.linalg.norm(cv)
    ma = None
    mi = None
    mas = None
    for i in ipts:
        iv = numpy.array(i) - tc
        ivn = iv / numpy.linalg.norm(iv)
        #a = numpy.arctan2(
        #    numpy.linalg.norm(numpy.cross(iv, cv)), numpy.dot(iv, cv))
        angle_sign = numpy.sign(numpy.cross(ivn, cvn))
        if numpy.sign(rspeed) != angle_sign:
            continue
        a = numpy.arccos(numpy.clip(numpy.dot(ivn, cvn), -1.0, 1.0))
        if ma is None or a < ma:
            ma = a
            mi = i
            mas = angle_sign
        #a = numpy.arccos(
        #    numpy.dot(iv, cv) /
        #    (numpy.linalg.norm(iv) * numpy.linalg.norm(cv)))
        #print(i, a)
    if ma is None:
        return c0
    #return mi
    pa = -mas * ma * step_ratio
    # rotate vector from tc to c0 (cv) by angle pa
    ca = numpy.cos(pa)
    sa = numpy.sin(pa)
    x, y = cv
    return (
        x * ca - y * sa + tc[0],
        x * sa + y * ca + tc[1])


def calculate_swing_target(
        tx, ty, z, leg_number, rspeed, step_ratio,
        min_hip_distance=None, target_calf_angle=0):
    # get x with vertical calf
    # vertical calf doesn't work with z > -19 or z < -75,
    # I don't think we can walk with
    # legs this high/low anyway
    # TODO cache these, they're used >1 time
    l, r = kinematics.leg.limits_at_z_2d(z)
    #c0x = kinematics.leg.x_with_vertical_calf(z)
    c0x = kinematics.leg.x_with_calf_angle(z, target_calf_angle)
    if c0x <= l or c0x >= r:
        c0x, _ = kinematics.leg.xy_center_at_z(z)
    # calculate target movement circle using center of tx, ty
    #tc = target_circle(tx, ty, c0x, 0.)
    tc = {
        'center': (tx, ty),
        'radius': numpy.sqrt((tx - c0x) ** 2. + (ty - 0.) ** 2.),
    }
    ipts = kinematics.leg.limit_intersections(
        tc, z, leg_number, min_hip_distance=min_hip_distance)
    sp = swing_position_from_intersections(
        [tx, ty], rspeed, [c0x, 0], ipts, step_ratio)
    return sp


class Foot(signaler.Signaler):
    def __init__(
            self, leg, cfg):
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
        self.swing_info = None
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
        #print("res.send_plan: [%s]%s" % (self.leg.leg_number, self.state))
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
            T = (
                self.leg_target *
                transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
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
            rx, ry, rspeed = self.swing_info
            sp = calculate_swing_target(
                rx, ry, self.cfg.lower_height,
                self.leg.leg_number, rspeed, self.cfg.step_ratio,
                min_hip_distance=self.cfg.min_hip_distance,
                target_calf_angle=self.cfg.target_calf_angle)
            self.swing_target = sp[0], sp[1]
            # print(self.swing_target, z)
            # TODO check if point is valid
            # TODO error out on invalid
            self.leg.send_plan(
                mode=consts.PLAN_TARGET_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    sp[0],
                    sp[1],
                    z),
                speed=self.cfg.get_speed('swing'))
        elif self.state == 'lower':
            v = -self.cfg.get_speed('lower')
            T = (
                self.leg_target *
                transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
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
            #self.swing_target = (
            #    self.cfg.foot_center['x'], self.cfg.foot_center['y'])
            # I won't know z until the leg is lifted
            #sp = calculate_swing_target(
            #    rx, ry, z, self.leg.leg_number, R[1], self.cfg.step_ratio,
            #    min_hip_distance=self.cfg.min_hip_distance)
            self.swing_info = (rx, ry, R[1])
            self.swing_target = None
            #self.swing_target = (
            #    self.center[0] + lx * self.cfg.step_size,
            #    self.center[1] + ly * self.cfg.step_size)
        self.send_plan()

    def set_state(self, state):
        if state != self.state and self.restriction is not None:
            # reset restriction smoothing
            self.restriction['dr'] = 0.
        self.state = state
        if self.state == 'lift':
            self.unloaded_height = None
            self.last_lift_time = time.time()
        self.send_plan()
        self.trigger('state', state)

    def calculate_restriction(self, xyz, angles):
        """Calculate leg restriction
        Result is stored in self.restriction and signaled as 'restriction'
        Result contains:
            - time: time of xyz event
            - r: current calculated restriction
            - idr: slope of restriction change from last to new value
            - dr: smoothed idr
        """
        # use angle limits to compute restriction
        r = 0
        for j in ('hip', 'thigh', 'knee'):
            jmin, jmax = self.limits[j]
            jmid = (jmax + jmin) / 2.
            if angles[j] > jmid:
                jl = angles[j] - jmax
            else:
                jl = jmin - angles[j]
            # take 'max' across joint angles, only the worst sets restriction
            r = max(min(1.0, numpy.exp(self.cfg.eps * jl)), r)
            #if self.leg.leg_number == 1:
            #    print(j, jl, r)
        # TODO use calf angle
        #cx, cy = self.center
        #d = ((cx - xyz['x']) ** 2. + (cy - xyz['y']) ** 2.) ** 0.5
        #r = numpy.exp(-self.r_eps * (d - self.radius))
        # TODO restriction modifier
        # add in the 'manual' restriction modifier (set from ui/controller)
        r += self.restriction_modifier
        if self.restriction is not None:
            pt = self.restriction['time']
            dt = (xyz['time'] - pt)
            idr = (r - self.restriction['r']) / dt
            dr = (
                self.restriction['dr'] * self.cfg.dr_smooth +
                idr * (1. - self.cfg.dr_smooth))
        else:  # if no previous value, can't calculate dr
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
                        (self.xyz['z'] - self.cfg.lower_height) <
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
                    self.xyz['z'] > (
                        self.unloaded_height + self.cfg.lift_height)):
                new_state = 'swing'
            #if self.xyz['z'] > self.lift_height:
            #    new_state = 'swing'
        # clear xyz and angles cache
        self.xyz = None
        self.angles = None
        if new_state is not None:
            print("setting new state: %s" % new_state)
            self.set_state(new_state)
