#!/usr/bin/env python
"""
Maybe start with the core computation (calculate_restriction):
    needs:
        xyz: foot position
        angles: joint angles
        limits: joint limits
        max_calf_angle
        min_hip_distance + buffer
        params for restriction components (eps, inflection)
        foot center (computed by)
            - lower height
            - target calf angle
            - max calf angle
            - xyz limits at z

Supply this a stance plan in body coordinates
it will produce plans in body coordinates
restriction will be updated with foot coordinates

it will produce 'requests' for plans that will be 'accepted'
"""

import time

import numpy

from .. import consts
#from .. import geometry
from .. import kinematics
from ..leg import plans
from .. import log
from .. import signaler
from .. import transforms


class Foot(signaler.Signaler):
    def __init__(
            self, leg, param):
        super(Foot, self).__init__()
        self.leg = leg
        self.param = param
        self.limits = leg.geometry.get_limits()
        self.logger = log.make_logger(
            'Res-%s' %
            consts.LEG_NAME_BY_NUMBER[self.leg.leg_number])
        self.leg.on('xyz', self.on_xyz)
        self.leg.on('angles', self.on_angles)
        self.last_lift_time = time.time()
        self.leg_target = None
        self.stance_plan = None
        self.body_target = None
        self.swing_target = None
        self.swing_info = (0, 0, 0)
        self.unloaded_height = None
        # stance -> lift -> swing -> lower -> wait
        self.state = None
        self.restriction = None
        #self.r0 = 0.
        #self.rr = 0.
        self.xyz = None
        self.angles = None
        self.restriction_modifier = 0.
        self.center_offset = (0, 0)
        self.halted = False

    def set_halt(self, value):
        self.halted = value

    def reset(self):
        self.restriction_modifier = 0.
        #self.r0 = 0.
        #self.rr = 0.
        self.center_offset = (0, 0)

    def calculate_swing_target(self):
        if self.unloaded_height is None:
            z = self.xyz['z'] + self.param['res.lift_height']
        else:
            z = self.unloaded_height + self.param['res.lift_height']
        min_hip_distance = (
                self.param['min_hip_distance'] +
                self.param['res.min_hip_buffer'])
        tcar = numpy.radians(self.param['res.target_calf_angle'])
        mcar = numpy.radians(self.param['res.max_calf_angle'])
        rx, ry, rspeed = self.swing_info
        sp = self.leg.geometry.xy_along_arc(
            rx, ry, self.param['res.lower_height'], rspeed,
            step_ratio=self.param['res.step_ratio'],
            min_hip_distance=min_hip_distance,
            target_calf_angle=tcar,
            max_calf_angle=mcar,
            x_offset=self.center_offset[0],
            y_offset=self.center_offset[1])
        return sp[0], sp[1], z

    def should_lift(self):
        if self.restriction_modifier > 0:
            return True
        # should the leg lift based on swing target > N in from current xyz
        sp = self.calculate_swing_target()
        dx = sp[0] - self.xyz['x']
        dy = sp[1] - self.xyz['y']
        d = numpy.sqrt(dx * dx + dy * dy)
        self.logger.debug({'should_lift': {
            'sp': sp,
            'dxy': (dx, dy),
            'd': d,
            'xyz': self.xyz,
            'return': d >= self.param['res.min_step_size'],
        }})
        return d >= self.param['res.min_step_size']

    #def _calculate_restriction(
    #        self,
    #        xyz, angles, limits, limit_eps, calf_eps, max_calf_angle,
    #        min_hip_distance, min_hip_eps, leg):
    def _calculate_restriction(self, xyz, angles):

        limits = self.limits
        max_calf_angle = numpy.radians(self.param['res.max_calf_angle'])
        leg = self.leg
        #nr = self._calculate_restriction(
        #    nxyz, nangles, self.limits, self.param['res.eps'],
        #    self.param['res.calf_eps'], mcar,
        #    min_hip_distance, self.param['res.min_hip_eps'],
        #    self.leg)
        # TODO log individual restriction values?
        # use angle limits to compute restriction
        r = 0
        #eps = numpy.log(limit_eps)
        eps = numpy.log(self.param['res.limit_eps'])
        for j in ('hip', 'thigh', 'knee'):
            jmin, jmax = limits[j]
            # limit to some percent of total limits
            jrange = (jmax - jmin) * self.param['res.limit_range']
            jmid = (jmax + jmin) / 2.
            jmin, jmax = jmid - jrange / 2, jmid + jrange / 2

            # inflection point
            ipt = (jmax - jmin) * self.param['res.limit_inflection_ratio']
            #jabsmax = float(max(
            #    abs(jmid - jmax), abs(jmin - jmid)))
            if angles[j] > jmid:
                jl = angles[j] - jmax
                ipt *= -1.0
            else:
                #jl = jmin - angles[j]
                jl = angles[j] - jmin
            # take 'max' across joint angles, only the worst sets restriction
            #r = max(min(1.0, numpy.exp(limit_eps * (jl / jabsmax))), r)
            #print(j, angles[j], (jmin, jmax), ipt, jmid, jl, eps, numpy.exp(eps/ipt * jl))
            r = max(min(1.0, numpy.exp(eps/ipt * jl)), r)
            #if self.leg.leg_number == 1:
            #    print(j, jl, r)

        # calf angle, if eps == 0, skip
        #if calf_eps > 0.001:
        #    ca = abs(leg.geometry.angles_to_calf_angle(
        #        angles['hip'], angles['thigh'], angles['knee']))
        #    cr = numpy.exp(calf_eps * ((ca - max_calf_angle) / max_calf_angle))
        calf_eps = numpy.log(self.param['res.calf_eps'])
        ca = abs(leg.geometry.angles_to_calf_angle(
            angles['hip'], angles['thigh'], angles['knee']))
        ipt = max_calf_angle * self.param['res.calf_inflection_ratio']
        cr = numpy.exp(calf_eps/ipt * ca)
        #cr = numpy.exp(calf_eps * ((ca - max_calf_angle) / max_calf_angle))
        r = max(min(1.0, cr), r)

        # minimum hip distance, if eps == 0, skip
        #if min_hip_eps > 0.001:
        #    hr = numpy.exp(
        #        min_hip_eps * ((min_hip_distance - xyz['x']) / min_hip_distance))
        #    r = max(min(1.0, hr), r)
        min_hip_distance = (
            self.param['min_hip_distance'] +
            self.param['res.min_hip_buffer'])
        hr = numpy.exp(numpy.log(self.param['res.min_hip_eps'])/min_hip_distance * xyz['x'])
        r = max(min(1.0, hr), r)

        # add restriction parameter for distance from foot center
        cx, cy, cz = self.compute_center_position()
        dx = abs(xyz['x'] - cx)
        dy = abs(xyz['y'] - cy)
        dr = numpy.sqrt(dx * dx + dy * dy)
        # exp(-log(0.1)/rt * (dr - 30))
        fcr = numpy.exp(
            -numpy.log(self.param['res.center_eps'])/self.param['res.center_inflection'] *
            (dr - self.param['res.center_radius']))
        r = max(min(1.0, fcr), r)
        return r

    def compute_center_position(self):
        """
        c0z: center z coordinate
        target_calf_angle (in radians)
        max_calf_angle (in radians)
        """
        # TODO cache this, only needs to change when inputs change
        c0z = self.param['res.lower_height']
        target_calf_angle = numpy.radians(self.param['res.target_calf_angle'])
        max_calf_angle = numpy.radians(self.param['res.max_calf_angle'])
        c0x, c0y = self.leg.geometry.xy_center_at_z(
            c0z, target_calf_angle, max_calf_angle,
            self.center_offset[0], self.center_offset[1])
        return c0x, c0y, c0z

    def send_plan(self):
        #print("res.send_plan: [%s]%s" % (self.leg.leg_number, self.state))
        if self.state is None or self.leg_target is None:
            # TODO always stop on disable?
            self.leg.send_plan(mode=consts.PLAN_STOP_MODE)
        elif self.state in ('stance', 'wait'):
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=self.leg_target,
                speed=0)
        elif self.state == 'lift':
            #v = self.get_mode_speed('lift')
            v = (
                self.param['speed.foot'] *
                self.param['speed.scalar'] *
                self.param['speed.lift_scale'])
            T = (
                self.leg_target *
                transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
            #print(self.leg_target, T)
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=T,
                speed=0)
        elif self.state == 'swing':
            self.swing_target = self.calculate_swing_target()
            # TODO check if point is valid
            # TODO error out on invalid
            self.leg.send_plan(
                mode=consts.PLAN_TARGET_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=(
                    self.swing_target[0],
                    self.swing_target[1],
                    self.swing_target[2]),
                speed=(
                    self.param['speed.foot'] *
                    self.param['speed.scalar'] *
                    self.param['speed.swing_scale']))
        elif self.state == 'lower':
            #v = -self.get_mode_speed('lower')
            v = -(
                self.param['speed.foot'] *
                self.param['speed.scalar'] *
                self.param['speed.lower_scale'])

            T = (
                self.leg_target *
                transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=T,
                speed=0)

    def set_target(self, target, update_swing=True):
        self.logger.debug({'set_target': (target, update_swing)})
        bx, by = target.rotation_center
        rx, ry, rz = kinematics.body.body_to_leg(
            self.leg.leg_number, bx, by, 0)
        lT = transforms.rotation_about_point_3d(
            rx, ry, rz, 0, 0, target.speed)
        # add z change
        if target.dz != 0.0:
            lT = lT * transforms.translation_3d(0, 0, target.dz)
        if update_swing:
            self.swing_info = (rx, ry, target.speed)
            self.swing_target = None
        self.leg_target = lT
        self.stance_plan = plans.Plan(
            mode=consts.PLAN_MATRIX_MODE,
            frame=consts.PLAN_LEG_FRAME,
            matrix=self.leg_target,
            speed=0)
        self.send_plan()

    def set_state(self, state):
        if state != self.state and self.restriction is not None:
            # reset restriction smoothing
            #print("resetting dr")
            self.restriction['dr'] = 0.
        self.state = state
        self.logger.debug({'state': state})
        if self.state == 'lift':
            self.unloaded_height = None
            self.last_lift_time = time.time()
        elif self.state == 'swing':
            pass
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
        mcar = numpy.radians(self.param['res.max_calf_angle'])
        min_hip_distance = (
            self.param['min_hip_distance'] +
            self.param['res.min_hip_buffer'])
        r = self._calculate_restriction(xyz, angles)
        #r = self._calculate_restriction(
        #    xyz, angles, self.limits, self.param['res.eps'],
        #    self.param['res.calf_eps'], mcar,
        #    min_hip_distance, self.param['res.min_hip_eps'],
        #    self.leg)
        # compute restriction for next location if > next_res_thresh away
        nxyz = plans.follow_plan(
            [xyz['x'], xyz['y'], xyz['z']], self.stance_plan)
        #if self.halted:
        #    nr = r
        #else:
        # TODO how to avoid 'noise' prediction nr != r
        nangles = self.leg.geometry.point_to_angles(*nxyz)
        nxyz = {'x': nxyz[0], 'y': nxyz[1], 'z': nxyz[2], 'time': xyz['time']}
        nangles = {
            'hip': nangles[0], 'thigh': nangles[1], 'knee': nangles[2],
            'time': angles['time']}
        nr = self._calculate_restriction(nxyz, nangles)
        #nr = self._calculate_restriction(
        #    nxyz, nangles, self.limits, self.param['res.eps'],
        #    self.param['res.calf_eps'], mcar,
        #    min_hip_distance, self.param['res.min_hip_eps'],
        #    self.leg)
        # add in the 'manual' restriction modifier (set from ui/controller)
        c0x, c0y, c0z = self.compute_center_position()
        r += self.restriction_modifier
        if self.restriction is not None:
            pt = self.restriction['time']
            dt = (xyz['time'] - pt)
            idr = (r - self.restriction['r']) / dt
            dr = (
                self.restriction['dr'] * self.param['res.dr_smooth'] +
                idr * (1. - self.param['res.dr_smooth']))
        else:  # if no previous value, can't calculate dr
            idr = 0.
            dr = 0.
        bc = kinematics.body.leg_to_body(self.leg.leg_number, c0x, c0y, c0z)
        self.restriction = {
            'time': xyz['time'], 'r': r, 'dr': dr, 'idr': idr,
            'nr': nr, 'state': self.state,
            #'rr': self.rr, 'r0': self.r0,
            'center': bc}
        self.logger.debug({'restriction': self.restriction})
        self.trigger('restriction', self.restriction)

    def _is_swing_done(self, xyz):
        tx, ty, _ = self.swing_target
        d = ((tx - xyz['x']) ** 2. + (ty - xyz['y']) ** 2.) ** 0.5
        # TODO also check for increase in distance
        return d < self.param['res.swing_slop']

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
            # check for loaded >L lbs
            if (
                    (
                        (self.xyz['z'] - self.param['res.lower_height']) <
                        self.param['res.height_slop']) and
                    self.angles['calf'] > self.param['res.loaded_weight']):
                new_state = 'wait'
                #if self.param['res.zero_on_lower']:
                #    self.r0 = self.rr
        elif self.state == 'wait':
            if self.restriction['nr'] > self.restriction['r']:
                new_state = 'stance'
                #self.r0 = 0.
            #if self.restriction['dr'] > self.param['res.wait_dr_thresh']:
            #    # print(
            #    #     "exiting wait[%s]: %s" %
            #    #     (self.leg.leg_number, self.restriction))
            #    new_state = 'stance'
        #elif self.state == 'stance'
        elif self.state == 'lift':
            # check for unloaded and >Z inches off ground
            if (
                    self.unloaded_height is None and
                    self.angles['calf'] < self.param['res.unloaded_weight']):
                self.unloaded_height = self.xyz['z']
            if (
                    self.unloaded_height is not None and
                    self.xyz['z'] > (
                        self.unloaded_height + self.param['res.lift_height'])):
                new_state = 'swing'
            #if self.xyz['z'] > self.lift_height:
            #    new_state = 'swing'
        # clear xyz and angles cache
        self.xyz = None
        self.angles = None
        if new_state is not None:
            #print(
            #    "setting new state[%s]: %s, %s" % (
            #        self.leg.leg_number, new_state, self.restriction))
            self.set_state(new_state)
