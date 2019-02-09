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
        self.min_hip_override = False
        self.mode = 'body_move'
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
        # self.conn = leg_teensy
        self.joy = joy
        if self.joy is not None:
            # self.joy.on('button', self.on_button)
            # self.joy.on('axis', self.on_axis)
            self.joy.on('buttons', self.on_buttons)
            self.joy.on('axes', self.on_axes)
        self.deadman = False

        #self.last_xyz = None
        #self.joy_smoothing = False
        #self.send_target_dt = 0.05
        #st = getattr(self.joy, 'settle_time', 1.)
        #self.update_target_until = time.time() - st
        #self.last_target_update = time.time()

        # stop all legs
        self.all_legs('set_estop', consts.ESTOP_DEFAULT)

        # monitor estop of all legs, broadcast when stopped
        for i in self.legs:
            self.legs[i].on('estop', lambda v, ln=i: self.on_leg_estop(v, ln))

        # check if this is the test leg in a box
        if len(self.legs) == 1 and 7 in self.legs:
            self.speeds = {
                'raw': 0.5,
                #'sensor': 65535,
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
        print("Leg estop: %s, %s" % (leg_number, value))
        if value:
            for i in self.legs:
                if i != leg_number and self.legs[i].estop == consts.ESTOP_OFF:
                    self.legs[i].set_estop(value)

    def set_mode(self, mode):
        if mode not in self.modes:
            raise Exception("Invalid mode: %s" % (mode, ))
        # always reset speed scalar
        self.set_speed(1.)
        # handle mode transitions
        if self.mode == 'body_restriction':
            self.res.disable()
        #elif self.mode == 'leg_calibration':
        #    self.calibrator.detach()
        self.mode = mode
        self.trigger('mode', mode)
        # handle mode transitions
        if self.mode == 'body_restriction':
            # TODO integrate this with enable
            for i in self.res.feet:
                self.res.feet[i].state = 'stance'
            self.res.enable(None)
            if self.deadman:
                self.set_target()
        elif self.mode == 'leg_pwm':
            self.all_legs('enable_pid', False)
        #elif self.mode == 'leg_calibration':
        #    self.calibrator.attach(self.leg)
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
            #if self.mode == 'leg_calibration':
            #    # disable calibration
            #    self.calibrator.detach()
        else:
            self.leg = self.legs[index]
            if self.mode == 'leg_calibration':
                # disable, then re-enable calibration
                self.attach(self.leg)
        self.trigger('set_leg', index)

    def on_buttons(self, buttons):
        if buttons.get('select', 0):  # advance mode
            mi = self.modes.index(self.mode)
            mi += 1
            if mi == len(self.modes):
                mi = 0
            self.set_mode(self.modes[mi])
        elif buttons.get('up', 0):
            # increase speed scalar
            self.set_speed(self.speed_scalar + self.speed_step)
            print("new speed: ", self.speed_scalar)
        elif buttons.get('down', 0):
            self.set_speed(self.speed_scalar - self.speed_step)
            print("new speed: ", self.speed_scalar)
        elif 'left' in buttons or 'right' in buttons:
            di = None
            if buttons.get('left', 0):
                di = -1
            if buttons.get('right', 0):
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
        elif 'one_right' in buttons:
            if buttons['one_right'] and not self.deadman:
                self.all_legs('set_estop', 0)
                if self.mode != 'leg_pwm':
                    self.all_legs('enable_pid', True)
                self.deadman = True
                #self.joy.reset_smoothing(thumb_mid)
                #self.last_xyz = None
                self.set_target()
            elif not buttons['one_right'] and self.deadman:
                self.all_legs('set_estop', 1)
                self.all_legs('stop')
                self.deadman = False
                #self.update_target_until = time.time() - 1.0
        elif buttons.get('square', 0):
            print(self.leg.loop_time_stats)
        #elif event['name'] == 'circle':
        #    if self.mode == 'leg_calibration':
        #        self.calibrator.set_subroutine('sensors', 'thigh')
        #elif event['name'] == 'cross':
        #    if self.mode == 'leg_calibration':
        #        self.calibrator.set_subroutine('sensors', 'knee')
        elif buttons.get('triangle', 0):
            print("Resetting loop time stats")
            self.leg.loop_time_stats.reset()

    def on_axes(self, axes):
        # check if target vector has changed > some amount
        # if so, read and send new target
        if any((n in axes for n in (
                'thumb_left_x', 'thumb_left_y',
                'thumb_right_x', 'thumb_right_y',
                #'one_left', 'two_left',
                ))):
            if self.deadman:
                self.set_target()
                # continue updating target every 0.1 seconds
                # for another 0.5 seconds
                #self.last_target_update = time.time()
                #if self.joy_smoothing:
                #    self.update_target_until = (
                #        self.last_target_update +
                #        self.joy.settle_time + self.send_target_dt)
        if 'cross' in axes and self.mode == 'body_restriction':
            # add restriction to current leg
            if self.leg is not None:
                foot = self.res.feet[self.leg.leg_number]
                foot.restriction_modifier = axes['cross'] / 255.
                print("%s" % foot.restriction_modifier)

    """
    def on_button(self, event):
        if event['name'] == 'select' and event['value']:  # advance mode
            mi = self.modes.index(self.mode)
            mi += 1
            if mi == len(self.modes):
                mi = 0
            self.set_mode(self.modes[mi])
        elif event['name'] == 'up':
            # increase speed scalar
            self.set_speed(self.speed_scalar + self.speed_step)
            print("new speed: ", self.speed_scalar)
        elif event['name'] == 'down':
            self.set_speed(self.speed_scalar - self.speed_step)
            print("new speed: ", self.speed_scalar)
        elif event['name'] in ('left', 'right') and event['value']:
            di = ('left', None, 'right').index(event['name']) - 1
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
        elif event['name'] == 'one_right':
            if event['value'] and not self.deadman:
                self.all_legs('set_estop', 0)
                if self.mode != 'leg_pwm':
                    self.all_legs('enable_pid', True)
                self.deadman = True
                #self.joy.reset_smoothing(thumb_mid)
                #self.last_xyz = None
                self.set_target()
            elif not event['value'] and self.deadman:
                self.all_legs('set_estop', 1)
                self.all_legs('stop')
                self.deadman = False
                #self.update_target_until = time.time() - 1.0
        elif event['name'] == 'square':
            print(self.leg.loop_time_stats)
        #elif event['name'] == 'circle':
        #    if self.mode == 'leg_calibration':
        #        self.calibrator.set_subroutine('sensors', 'thigh')
        #elif event['name'] == 'cross':
        #    if self.mode == 'leg_calibration':
        #        self.calibrator.set_subroutine('sensors', 'knee')
        elif event['name'] == 'triangle':
            print("Resetting loop time stats")
            self.leg.loop_time_stats.reset()

    def on_axis(self, event):
        # check if target vector has changed > some amount
        # if so, read and send new target
        if event['name'] in (
                'thumb_left_x', 'thumb_left_y',
                'thumb_right_x', 'thumb_right_y',
                'one_left', 'two_left'):
            if self.deadman:
                self.set_target()
                # continue updating target every 0.1 seconds
                # for another 0.5 seconds
                #self.last_target_update = time.time()
                #if self.joy_smoothing:
                #    self.update_target_until = (
                #        self.last_target_update +
                #        self.joy.settle_time + self.send_target_dt)
        if event['name'] == 'cross' and self.mode == 'body_restriction':
            # add restriction to current leg
            if self.leg is not None:
                foot = self.res.feet[self.leg.leg_number]
                foot.restriction_modifier = event['value'] / 255.
                print("%s" % foot.restriction_modifier)
    """

    def set_target(self):
        # from joystick
        """
        az = (
            self.joy.axes.get('one_left', 0) -
            self.joy.axes.get('two_left', 0))
        if abs(az) < thumb_db:
            az = 0
        else:
            if az > 0:
                az -= thumb_db
            else:
                az += thumb_db
        """
        #if self.joy_smoothing is False or self.mode == 'leg_pwm':
        lx = self.joy.axes.get('thumb_left_x', thumb_mid) - thumb_mid
        ly = self.joy.axes.get('thumb_left_y', thumb_mid) - thumb_mid
        rx = self.joy.axes.get('thumb_right_x', thumb_mid) - thumb_mid
        ry = self.joy.axes.get('thumb_right_y', thumb_mid) - thumb_mid
        # TODO remove thumb_db from joystick?
        if abs(lx) < thumb_db:
            lx = 0
        if abs(ly) < thumb_db:
            ly = 0
        if abs(rx) < thumb_db:
            rx = 0
        if abs(ry) < thumb_db:
            ry = 0
        #else:
        #    ax = self.get_axis('thumb_left_x')
        #    ay = self.get_axis('thumb_left_y')
        #    aa = self.get_axis('thumb_right_x')
        #    ab = self.get_axis('thumb_right_y')
        #if ax == 0 and ay == 0 and az == 0:
        #    return
        lx = max(-1., min(1., lx / float(thumb_scale)))
        ly = max(-1., min(1., -ly / float(thumb_scale)))
        #az = max(-1., min(1., az / 255.))
        rx = max(-1., min(1., rx / float(thumb_scale)))
        ry = max(-1., min(1., -ry / float(thumb_scale)))
        #xyz = (lx, ly, az)
        xyz = (lx, ly, ry)
        #if self.last_xyz is not None:
        #    if (
        #            numpy.sum(numpy.abs(
        #                numpy.array(self.last_xyz) - numpy.array(xyz)))
        #            < 0.01):
        #        return
        #self.last_xyz = xyz
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
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=xyz, speed=speed)
            #print("sending plan: %s, %s" % (xyz, speed))
            #self.leg.send_plan(
            #    mode=consts.PLAN_ARC_MODE,
            #    frame=consts.PLAN_LEG_FRAME,
            #    linear=[0, 0, 0],
            #    angular=xyz, speed=0.01)
        elif self.mode == 'leg_body':
            if self.leg is None:
                return
            speed = self.speed_scalar * self.speeds['body']
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_BODY_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_calibration':
            pass
        elif self.mode == 'body_move':
            #if self.joy.keys.get('circle', 0) == 0:
            #if self.joy.keys.get('one_left', 0) == 0:
            if self.joy.buttons.get('one_left', 0) == 0:
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
                #speed = self.speed_scalar * 0.01
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
            #self.res.set_target(numpy.array([rx, ly, az]))
            #omni_walk = bool(self.joy.keys.get('one_left', 0))
            omni_walk = bool(self.joy.buttons.get('one_left', 0))
            # convert joystick input to a rotation about some point
            # need to calculate
            # - center of rotation (in body coordinates)
            # - linear speed during rotation
            # TODO add dz
            dz = 0.
            if omni_walk:
                # calc direction by lx, ly
                # rotate 90 for radius
                a = numpy.arctan2(-lx, ly)
                crx = numpy.cos(a) * max_radius
                cry = numpy.sin(a) * max_radius
                # set speed by magnitude
                m = numpy.linalg.norm([lx, ly])
                rs = self.res.calc_stance_speed((crx, cry), m)
            else:
                # y center of rotation is always 0
                crx = axis_to_radius(rx)
                cry = 0.
                rs = (
                    self.res.calc_stance_speed((crx, cry), ly)
                    * numpy.sign(crx))
            self.res.set_target(
                restriction.body.BodyTarget((crx, cry), rs, dz))

    def update(self):
        if self.joy is not None:
            self.joy.update()
        #if (
        #        self.joy_smoothing and
        #        self.last_target_update <= self.update_target_until):
        #    t = time.time()
        #    if (t - self.last_target_update) >= self.send_target_dt:
        #        self.set_target()
        #        self.last_target_update = t
        #if self.mode == 'leg_calibration':
        #    pass
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
