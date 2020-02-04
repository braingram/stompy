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

import math
import time

from .. import consts
#from .. import geometry
from .. import kinematics
from ..leg import plans
from .. import log
from .. import signaler
from .. import transforms


class LegTarget(object):
    def __init__(self, body_target, leg_number):   # make from body target
        # store ref to body target
        self.body_target = body_target

        # transform rotation to leg coordinates
        bx, by = body_target.rotation_center
        rx, ry, rz = kinematics.body.body_to_leg(
            leg_number, bx, by, 0)
        lT = transforms.rotation_about_point_3d(
            rx, ry, rz, 0, 0, body_target.speed)

        # add z change
        if body_target.dz != 0.0:
            lT = lT * transforms.translation_3d(0, 0, body_target.dz)
        
        self.leg_matrix = lT
        self.swing_info = (rx, ry, body_target.speed)

        self.stance_plan = plans.Plan(
            mode=consts.PLAN_MATRIX_MODE,
            frame=consts.PLAN_LEG_FRAME,
            matrix=self.leg_matrix,
            speed=0)

    # TODO move swing_target to here?


class Foot(signaler.Signaler):
    def __init__(
            self, leg, param):
        super(Foot, self).__init__()
        self._cache = {}
        self.leg = leg
        self.param = param
        self.limits = leg.geometry.get_limits()
        self.logger = log.make_logger(
            'Res-%s' %
            consts.LEG_NAME_BY_NUMBER[self.leg.leg_number])
        self.leg.on('xyz', self.on_xyz)
        self.leg.on('angles', self.on_angles)
        self.last_lift_time = time.time()

        self.leg_target = None  # target in leg coordinates
        self.swing_target = None  # swing target location (x, y, z)

        self.unloaded_height = None
        # stance -> lift -> swing -> lower -> wait
        self.state = None
        self.restriction = None
        self.xyz = None
        self.angles = None
        self.restriction_modifier = 0.
        self.center_offset = (0, 0)
        self.halted = False

    def set_halt(self, value):
        self.halted = value
        self.send_plan()

    def reset(self):
        self.restriction_modifier = 0.
        self.center_offset = (0, 0)

    def calculate_swing_target(self):
        if self.unloaded_height is None:
            z = self.xyz['z'] + self.param['res.lift_height']
        else:
            z = self.unloaded_height + self.param['res.lift_height']
        min_hip_distance = (
                self.param['min_hip_distance'] +
                self.param['res.fields.min_hip.buffer'])
        tcar = math.radians(self.param['res.target_calf_angle'])
        mcar = math.radians(self.param['res.fields.calf_angle.max'])
        rx, ry, rspeed = self.leg_target.swing_info
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
        d = math.sqrt(dx * dx + dy * dy)
        self.logger.debug({'should_lift': {
            'sp': sp,
            'dxy': (dx, dy),
            'd': d,
            'xyz': self.xyz,
            'return': d >= self.param['res.min_step_size'],
        }})
        return d >= self.param['res.min_step_size']
    
    def _in_cache(self, attr, *params):
        current_params = {self.param[p] for p in params}

        def cache_function(f, current_params=current_params, attr=attr):
            self._cache[attr] = (current_params, f)
            return f

        if attr not in self._cache:  # not in cache
            return False, cache_function
        params, func = self._cache[attr]
        if params == current_params:  # cache matches params
            return True, func
        return False, cache_function

    def calculate_joint_angle_restriction(self, angles):
        r = {}
        for jn in consts.JOINT_NAMES:
            cr, cf = self._in_cache(
                jn + '_angle',
                'res.fields.joint_angle.eps',
                'res.fields.joint_angle.inflection',
                'res.fields.joint_angle.range')
            if not cr:
                eps = math.log(self.param['res.fields.joint_angle.eps'])
                inflection = self.param['res.fields.joint_angle.inflection']
                range_ratio = self.param['res.fields.joint_angle.range']

                # actual min/max
                jmin, jmax = self.limits[jn]
                
                # limit range
                jr = (jmax - jmin) * range_ratio
                # centered on midpoint
                jmid = (jmax + jmin) / 2.
                # limited min/max
                jmin, jmax = jmid - jr / 2, jmid + jr / 2

                # inflection point
                ipt = jr * inflection

                # function needs:
                v = eps/ipt
                jr2 = jr / 2
                cf = cf(
                    lambda a, v=v, jr2=jr2, jmid=jmid:
                    min(1.0, math.exp(v * (jr2 - abs(a - jmid)))))
            r[jn] = cf(angles[jn])
        r['r'] = max(r.values())
        return r

    def calculate_calf_angle_restriction(self, angles):
        cr, cf = self._in_cache(
            'calf_angle',
            'res.fields.calf_angle.max',
            'res.fields.calf_angle.eps',
            'res.fields.calf_angle.inflection')
        if not cr:
            max_calf_angle = math.radians(self.param['res.fields.calf_angle.max'])
            calf_eps = math.log(self.param['res.fields.calf_angle.eps'])
            ipt = max_calf_angle * self.param['res.fields.calf_angle.inflection']
            v = calf_eps / ipt
            cf = cf(lambda ca, v=v: min(1.0, math.exp(v * ca)))

        ca = abs(self.leg.geometry.angles_to_calf_angle(
                angles['hip'], angles['thigh'], angles['knee']))
        r = cf(ca)
        return {'r': r, 'calf_angle': ca}

    def calculate_hip_distance_restriction(self, xyz):
        cr, cf = self._in_cache(
            'hip_distance',
            'min_hip_distance',
            'res.fields.min_hip_buffer',
            'res.fields.min_hip_eps')
        if not cr:
            min_hip_distance = (
                self.param['min_hip_distance'] +
                self.param['res.fields.min_hip.buffer'])
            v = math.log(self.param['res.fields.min_hip.eps'])/min_hip_distance
            cf = cf(lambda x, v=v: min(1.0, math.exp(v * x)))
        r = cf(xyz['x'])
        return {'r': r}

    def calculate_foot_center_restriction(self, xyz):
        cr, cf = self._in_cache(
            'foot_center',
            'res.fields.center.eps',
            'res.fields.center.inflection',
            'res.fields.center.radius')
        if not cr:
            v = (
                    -math.log(self.param['res.fields.center.eps']) /
                    self.param['res.fields.center.inflection'])
            c = self.param['res.fields.center.radius']
            cf = cf(lambda d, v=v, c=c: min(1.0, math.exp((d - c) * v)))
        cx, cy, cz = self.calculate_center_position()
        dx = (xyz['x'] - cx)
        dy = (xyz['y'] - cy)
        dr = math.sqrt(dx * dx + dy * dy)
        r = cf(dr)
        return {'r': r, 'center': (cx, cy, cz)}
    
    def calculate_restriction(self, xyz, angles):
        info = {
            'joint_angle': self.calculate_joint_angle_restriction(angles),
            'calf_angle': self.calculate_calf_angle_restriction(angles),
            'hip_distance': self.calculate_hip_distance_restriction(xyz),
            'foot_center': self.calculate_foot_center_restriction(xyz),
        }
        info['r'] = max([info[k]['r'] for k in info])
        return info

    def calculate_center_position(self):
        """
        c0z: center z coordinate
        target_calf_angle (in radians)
        max_calf_angle (in radians)
        """
        # TODO cache this, only needs to change when inputs change
        c0z = self.param['res.lower_height']
        target_calf_angle = math.radians(self.param['res.target_calf_angle'])
        max_calf_angle = math.radians(self.param['res.fields.calf_angle.max'])
        c0x, c0y = self.leg.geometry.xy_center_at_z(
            c0z, target_calf_angle, max_calf_angle,
            self.center_offset[0], self.center_offset[1])
        return c0x, c0y, c0z

    def send_plan(self):
        #print("res.send_plan: [%s]%s" % (self.leg.leg_number, self.state))
        if self.state is None or self.leg_target is None:
            # TODO always stop on disable?
            return self.leg.send_plan(mode=consts.PLAN_STOP_MODE)
        if self.state == 'swing':
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
        else:  # not swing [stance, wait, lower, lift]
            if self.halted:
                T = transforms.translation_3d(0, 0, self.leg_target.body_target.dz)
            else:
                T = self.leg_target.leg_matrix
            if self.state == 'lift':
                v = (
                    self.param['speed.foot'] *
                    self.param['speed.scalar'] *
                    self.param['speed.lift_scale'])
                T = (
                    T *
                    transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
            elif self.state == 'lower':
                v = -(
                    self.param['speed.foot'] *
                    self.param['speed.scalar'] *
                    self.param['speed.lower_scale'])
                T = (
                    T *
                    transforms.translation_3d(0, 0, v * consts.PLAN_TICK))
            self.leg.send_plan(
                mode=consts.PLAN_MATRIX_MODE,
                frame=consts.PLAN_LEG_FRAME,
                matrix=T,
                speed=0)
        return

    def set_target(self, target):
        self.logger.debug({'set_target': target})
        self.leg_target = LegTarget(target, self.leg.leg_number)
        self.send_plan()

    def set_state(self, state):
        self.state = state
        self.logger.debug({'state': state})
        if self.state == 'lift':
            self.unloaded_height = None
            self.last_lift_time = time.time()
        elif self.state == 'swing':
            pass
        self.send_plan()
        self.trigger('state', state)

    def update_restriction(self, xyz, angles):
        """Calculate leg restriction
        Result is stored in self.restriction and signaled as 'restriction'
        Result contains:
            - time: time of xyz event
            - r: current calculated restriction
        """
        # calculate current restriction
        rinfo = self.calculate_restriction(xyz, angles)
        r = rinfo['r']

        # compute restriction for next location if > next_res_thresh away
        if self.leg_target is None:
            nextrinfo = rinfo
            nr = r
        else:
            # avoid 'noise' prediction nr != r by always using stance plan
            nxyz = plans.follow_plan(
                [xyz['x'], xyz['y'], xyz['z']], self.leg_target.stance_plan)
            nangles = self.leg.geometry.point_to_angles(*nxyz)
            nxyz = {'x': nxyz[0], 'y': nxyz[1], 'z': nxyz[2], 'time': xyz['time']}
            nangles = {
                'hip': nangles[0], 'thigh': nangles[1], 'knee': nangles[2],
                'time': angles['time']}
            nextrinfo = self.calculate_restriction(nxyz, nangles)
            nr = nextrinfo['r']

        # add in the 'manual' restriction modifier (set from ui/controller)
        r += self.restriction_modifier
        nr += self.restriction_modifier

        # add body center to restriction for display
        c0x, c0y, c0z = rinfo['foot_center']['center']
        bc = kinematics.body.leg_to_body(self.leg.leg_number, c0x, c0y, c0z)

        self.restriction = {
            'time': xyz['time'], 'r': r,
            'nr': nr, 'state': self.state,
            'center': bc,
            'rinfo': rinfo, 'nextrinfo': nextrinfo}
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
        self.update_restriction(self.xyz, self.angles)
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
        elif self.state == 'wait':
            if self.restriction['nr'] > self.restriction['r']:
                #print(
                #    "[%s] r: %0.2f; nr: %0.2f; delta: %0.4f" %
                #    (
                #        self.leg.leg_number,
                #        self.restriction['r'], self.restriction['nr'],
                #        self.restriction['nr'] - self.restriction['r']))
                new_state = 'stance'
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
        # clear xyz and angles cache
        self.xyz = None
        self.angles = None
        if new_state is not None:
            self.set_state(new_state)
