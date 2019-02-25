#!/usr/bin/env python
"""
modes: [select button]
    - leg
        - raw pwm
        - sensor frame [cross]
        - leg frame [circle]
        - body frame [triangle]
        - restriction? [square]
    - body
        - move body
        - position legs
        - restriction
parameters:
    - speed scalar [up/down]
    - leg index [left/right button]
    - target [thumb_left_x/y, one_left/two_left]
controls:
    - deadman
"""

import time

import numpy

#from . import calibrator
from .. import consts
from .. import leg
from .. import log
from .. import restriction
from .. import signaler


thumb_mid = 130
thumb_db = 10  # +-
thumb_scale = max(255 - thumb_mid, thumb_mid)

leg_select_buttons = ['leg_select_%i' % i for i in range(1, 7)]

max_radius = 100000.


def axis_to_radius(axis):
    if numpy.abs(axis) < 0.001:  # sign(0.0) == 0.
        radius = max_radius
    else:
        radius = (
            numpy.sign(axis) * max_radius /
            2. ** (numpy.log2(max_radius) * numpy.abs(axis)))
    return radius


class MultiLeg(signaler.Signaler):
    modes = [
        'leg_pwm',
        'leg_sensor',
        'leg_leg',
        'leg_body',
        #'leg_calibration',
        #'leg_restriction',
        'body_move',
        'sit_stand',
        #'body_position_legs',
        'body_restriction',
    ]

    def __init__(self, legs, joy, bodies):
        super(MultiLeg, self).__init__()
        self.legs = legs
        self.bodies = bodies
        #self.calibrator = calibrator.CalibrationRoutine()
        self.res = restriction.body.Body(legs)
        self.leg_index = sorted(legs)[0]
        self.leg = self.legs[self.leg_index]
        # if a foot gets within this distance of leg 0, throw an estop
        self.min_hip_distance = 25.0
        self.prevent_leg_xy_when_loaded = True
        self.min_hip_override = False
        self.mode = 'sit_stand'
        self.speeds = {
            'raw': 0.5,
            'sensor': 1200,
            'leg': 3.0,
            'body': 3.0,
            'body_angular': 0.05,
        }
        self.speed_scalar = 1.0
        self.speed_step = 0.05
        self.speed_scalar_range = (0.1, 2.0)
        self.height = numpy.nan
        self.joy = joy
        if self.joy is not None:
            self.joy.on('buttons', self.on_buttons)
            self.joy.on('axes', self.on_axes)
        self.deadman = False

        # stop all legs
        self.all_legs('set_estop', consts.ESTOP_DEFAULT)

        # monitor estop of all legs, broadcast when stopped
        for i in self.legs:
            self.legs[i].on('estop', lambda v, ln=i: self.on_leg_estop(v, ln))
            self.legs[i].on('xyz', lambda v, ln=i: self.on_leg_xyz(v, ln))

        # check if this is the test leg in a box
        if len(self.legs) == 1 and 7 in self.legs:
            self.speeds = {
                'raw': 0.5,
                'sensor': 4800,
                'leg': 3.0,
                'body': 3.0,
                'body_angular': 0.05,
            }

        # check if all legs are simulated
        if all([
                isinstance(self.legs[k], leg.teensy.FakeTeensy)
                for k in self.legs]):
            self.speeds['leg'] = 18
            self.speeds['body'] = self.speeds['leg']
            self.res.cfg.speeds.update({
                'stance': 12,
                'swing': 12,
                'lift': 12,
                'lower': 12,
            })
            self.set_mode('body_restriction')
            self.res.cfg.max_feet_up = 1
            self.res.cfg.speed_by_restriction = True

    def set_speed(self, speed):
        old_speed = self.speed_scalar
        self.speed_scalar = max(
            self.speed_scalar_range[0],
            min(self.speed_scalar_range[1], speed))
        if old_speed == self.speed_scalar:
            return
        log.info({"set_speed": self.speed_scalar})
        self.trigger('speed', self.speed_scalar)
        # TODO resend target?
        if self.mode == 'body_restriction':
            self.res.set_speed(self.speed_scalar)

    def on_leg_estop(self, value, leg_number):
        self.trigger('estop', value)
        if value:
            for i in self.legs:
                if i != leg_number and self.legs[i].estop == consts.ESTOP_OFF:
                    self.legs[i].set_estop(value)

    def on_leg_xyz(self, xyz, leg_number):
        # find lowest 3 legs (most negative)
        self.height = -numpy.mean(
            sorted([
                self.legs[i].xyz.get('z', numpy.nan) for i in self.legs])[:3])
        self.trigger('height', self.height)

    def set_mode(self, mode):
        if mode not in self.modes:
            raise Exception("Invalid mode: %s" % (mode, ))
        # always reset speed scalar
        self.set_speed(1.)
        # handle mode transitions
        if self.mode == 'body_restriction':
            self.res.disable()
        self.mode = mode
        self.trigger('mode', mode)
        # handle mode transitions
        if self.mode == 'body_restriction':
            # TODO integrate this with enable
            for i in self.res.feet:
                self.res.feet[i].state = 'stance'
            self.res.enable(None)
            if (
                    self.res.cfg.set_height_on_mode_select and
                    numpy.isfinite(self.height) and
                    (self.res.cfg.min_lower_height < -self.height) and
                    (-self.height < self.res.cfg.max_lower_height)):
                self.res.cfg.lower_height = -self.height
                self.trigger('config_updated')
            if self.deadman:
                self.set_target()
        elif self.mode == 'leg_pwm':
            self.all_legs('enable_pid', False)
        else:
            self.all_legs('stop')

    def all_legs(self, cmd, *args, **kwargs):
        log.info({"all_legs": (cmd, args, kwargs)})
        for leg in self.legs:
            getattr(self.legs[leg], cmd)(*args, **kwargs)

    def set_leg(self, index):
        log.info({"set_leg": index})
        self.leg_index = index
        # reset restriction modifier for leg
        if self.leg is not None:
            self.res.feet[self.leg.leg_number].restriction_modifier = 0.
        if self.leg_index is None:
            self.leg = None
        else:
            self.leg = self.legs[index]
            if self.mode == 'leg_calibration':
                # disable, then re-enable calibration
                self.attach(self.leg)
        self.trigger('set_leg', index)

    def on_buttons(self, buttons):
        if 'leg_mode_switch' in buttons:
            if buttons['leg_mode_switch']:
                self.set_mode('leg_body')
            else:
                if 'body_walk_switch' not in buttons:
                    buttons['body_walk_switch'] = (
                        self.joy.buttons.get('body_walk_switch', 0))
        if 'body_walk_switch' in buttons:
            if not self.joy.buttons.get('leg_mode_switch', 0):
                if buttons['body_walk_switch']:
                    self.set_mode('body_restriction')
                else:
                    self.set_mode('body_move')
        for (ln, bn) in enumerate(leg_select_buttons):
            if bn in buttons:
                ln += 1
                if buttons.get(bn, 0) and ln in self.legs:
                    self.set_leg(ln)
        if buttons.get('use_sliders_switch', 0):
            self._set_speed_by_slider()
            self._set_height_by_slider()
        if buttons.get('mode_inc', 0):  # advance mode
            mi = self.modes.index(self.mode)
            mi += 1
            if mi == len(self.modes):
                mi = 0
            self.set_mode(self.modes[mi])
        if buttons.get('speed_inc', 0):
            # increase speed scalar
            self.set_speed(self.speed_scalar + self.speed_step)
            print("new speed: ", self.speed_scalar)
        if buttons.get('speed_dec', 0):
            self.set_speed(self.speed_scalar - self.speed_step)
            print("new speed: ", self.speed_scalar)
        if 'leg_index_inc' in buttons or 'leg_index_dec' in buttons:
            di = None
            if buttons.get('leg_index_dec', 0):
                di = -1
            if buttons.get('leg_index_inc', 0):
                di = 1
            if di is not None:
                inds = sorted(self.legs)
                if self.leg_index is None:
                    i = 0
                else:
                    si = inds.index(self.leg_index) + di
                    if si == len(inds):
                        si = 0
                    elif si < 0:
                        si = len(inds) - 1
                    i = inds[si]
                self.set_leg(i)
        if 'deadman' in buttons:
            if buttons['deadman'] and not self.deadman:
                self.all_legs('set_estop', 0)
                if self.mode != 'leg_pwm':
                    self.all_legs('enable_pid', True)
                self.deadman = True
                self.set_target()
            elif not buttons['deadman'] and self.deadman:
                self.all_legs('set_estop', 1)
                self.all_legs('stop')
                self.deadman = False
        if 'restrict_leg' in buttons and self.mode == 'body_restriction':
            # add restriction to current leg
            if self.leg is not None:
                foot = self.res.feet[self.leg.leg_number]
                foot.restriction_modifier = buttons['restrict_leg']
        if buttons.get('report_stats', 0):
            print(self.leg.loop_time_stats)
        if buttons.get('reset_stats', 0):
            print("Resetting loop time stats")
            self.leg.loop_time_stats.reset()

    def _set_speed_by_slider(self):
        self.speed_scalar = self.joy.axes.get('speed_axis', 0) / 255.
        self.set_speed(self.speed_scalar)
        # TODO update ui

    def _set_height_by_slider(self):
        if 'height_axis' not in self.joy.axes:
            return
        h = self.joy.axes['height_axis'] / 255.
        h = (
            h * (
                self.res.cfg.min_lower_height -
                self.res.cfg.max_lower_height) +
            self.res.cfg.max_lower_height)
        return
        self.res.cfg.lower_height = h
        self.trigger('config_updated')

    def on_axes(self, axes):
        # check if target vector has changed > some amount
        # if so, read and send new target
        if any((n in axes for n in ('x', 'y', 'z'))):
            if self.deadman:
                self.set_target()
        if 'speed_axis' in axes:
            if self.joy.buttons.get('use_sliders_switch', 0):
                self._set_speed_by_slider()
        if 'height_axis' in axes:
            if self.joy.buttons.get('use_sliders_switch', 0):
                self._set_height_by_slider()

    def set_target(self):
        # from joystick
        xyz = []
        for axis in ('x', 'y', 'z'):
            jv = self.joy.axes.get(axis, thumb_mid) - thumb_mid
            if abs(jv) < thumb_db:
                jv = 0
            jv = max(-1., min(1., jv / float(thumb_scale)))
            xyz.append(jv)
        if self.mode == 'leg_pwm':
            if self.leg is None:
                return
            speed = self.speed_scalar * self.speeds['raw']
            self.leg.set_pwm(
                xyz[0] * speed,
                xyz[1] * speed,
                xyz[2] * speed)
        elif self.mode == 'leg_sensor':
            if self.leg is None:
                return
            speed = self.speed_scalar * self.speeds['sensor']
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_SENSOR_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_leg':
            if self.leg is None:
                return
            speed = self.speed_scalar * self.speeds['leg']
            if (
                    self.prevent_leg_xy_when_loaded and
                    (self.leg.angles['calf'] > self.res.cfg.loaded_weight)):
                xyz = [0., 0., xyz[2]]
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_body':
            if self.leg is None:
                return
            speed = self.speed_scalar * self.speeds['body']
            if (
                    self.prevent_leg_xy_when_loaded and
                    (self.leg.angles['calf'] > self.res.cfg.loaded_weight)):
                xyz = [0., 0., xyz[2]]
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_BODY_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_calibration':
            pass
        elif self.mode == 'sit_stand':
            speed = self.speed_scalar * self.speeds['body']
            xyz = [0, 0, xyz[2]]
            plan = {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_BODY_FRAME,
                'linear': -numpy.array(xyz),
                'speed': speed,
            }
            self.all_legs('send_plan', **plan)
        elif self.mode == 'body_move':
            if self.joy.buttons.get('sub_mode', 0) == 0:
                speed = self.speed_scalar * self.speeds['body']
                plan = {
                    'mode': consts.PLAN_VELOCITY_MODE,
                    'frame': consts.PLAN_BODY_FRAME,
                    'linear': -numpy.array(xyz),
                    'speed': speed,
                }
            else:
                # swap x and y
                xyz = [xyz[1], xyz[0], xyz[2]]
                speed = self.speed_scalar * self.speeds['body_angular']
                plan = {
                    'mode': consts.PLAN_ARC_MODE,
                    'frame': consts.PLAN_BODY_FRAME,
                    'linear': (0, 0, 0),
                    'angular': -numpy.array(xyz),
                    'speed': speed}
                print("Body rotation plan: %s" % (plan, ))
            self.all_legs('send_plan', **plan)
        elif self.mode == 'body_position_legs':
            pass
            # TODO
        elif self.mode == 'body_restriction':
            # pass in rx, ly, az
            # also pass in mode for crab walking
            omni_walk = bool(self.joy.buttons.get('sub_mode', 0))
            # convert joystick input to a rotation about some point
            # need to calculate
            # - center of rotation (in body coordinates)
            # - linear speed during rotation
            # TODO add dz
            dz = 0.
            if omni_walk:
                # calc direction by lx, ly
                # rotate 90 for radius
                #a = numpy.arctan2(-lx, ly)
                a = numpy.arctan2(-xyz[0], xyz[1])
                crx = numpy.cos(a) * max_radius
                cry = numpy.sin(a) * max_radius
                # set speed by magnitude
                m = numpy.linalg.norm([xyz[0], xyz[1]])
                rs = self.res.calc_stance_speed((crx, cry), m)
            else:
                # y center of rotation is always 0
                crx = axis_to_radius(xyz[0])
                cry = 0.
                # use both joystick x and y to determine 'speed'
                # to allow for turning in place
                if abs(xyz[0]) > abs(xyz[1]):
                    sv = abs(xyz[0]) * numpy.sign(xyz[1])
                else:
                    sv = xyz[1]
                rs = (
                    self.res.calc_stance_speed((crx, cry), sv)
                    * numpy.sign(crx))
            self.res.set_target(
                restriction.body.BodyTarget((crx, cry), rs, dz))

    def update(self):
        if self.joy is not None:
            self.joy.update()
        self.all_legs('update')
        if self.mode in ('body_move', 'body_restriction'):
            if self.min_hip_override:
                # check if override should be turned off
                disable_override = True
                threshold = self.min_hip_distance * 1.5
                for l in self.legs:
                    if self.legs[l].xyz.get('x', 0) < threshold:
                        disable_override = False
                if disable_override:
                    self.min_hip_override = False
                    # TODO trigger ui update?
                    self.trigger('config_updated')
            else:
                #
                # check if any foot has x < self.min_hip_distance
                all_stopped = True
                trigger_estop = False
                for l in self.legs:
                    if self.legs[l].estop == consts.ESTOP_OFF:  # enabled
                        all_stopped = False
                    if self.legs[l].xyz.get('x', 0) < self.min_hip_distance:
                        # set estop
                        trigger_estop = True
                # one of the legs is too close to the hip and not all
                # are stopped so trigger an estop on all legs
                if not all_stopped and trigger_estop:
                    print("estopping because foot too close to hip")
                    self.all_legs('set_estop', consts.ESTOP_DEFAULT)
        # update all body teensies
        [self.bodies[k].update() for k in self.bodies]
