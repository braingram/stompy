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

from . import calibrator
from .. import consts
from .. import leg
from .. import log
from .. import signaler


thumb_mid = 130
thumb_db = 5  # +-
thumb_scale = max(255 - thumb_mid, thumb_mid)


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

    def __init__(self, legs, joy):
        super(MultiLeg, self).__init__()
        self.legs = legs
        self.calibrator = calibrator.CalibrationRoutine()
        self.res = leg.restriction.Body(legs)
        self.leg_index = sorted(legs)[0]
        self.leg = self.legs[self.leg_index]
        self.mode = 'body_move'
        self.speeds = {
            'raw': 0.5,
            'sensor': 1200,
            'leg': 3.0,
            'body': 3.0,
        }
        self.speed_scalar = 1.0
        self.speed_step = 0.05
        self.speed_scalar_range = (0.1, 2.0)
        # self.conn = leg_teensy
        self.joy = joy
        if self.joy is not None:
            self.joy.on('button', self.on_button)
            self.joy.on('axis', self.on_axis)
        self.deadman = False
        self.joystick_move_throttle = 0.25
        self.last_joystick_move = time.time()
        self.reset_joystick_move_throttle()

        # stop all legs
        self.all_legs('set_estop', consts.ESTOP_DEFAULT)

        # monitor estop of all legs, broadcast when stopped
        for i in self.legs:
            self.legs[i].on('estop', lambda v, ln=i: self.on_leg_estop(v, ln))

    def reset_joystick_move_throttle(self):
        self.last_joystick_move = (
            time.time() - self.joystick_move_throttle * 2.)

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
                if i != leg_number:
                    self.legs[i].set_estop(value)

    def set_mode(self, mode):
        if mode not in self.modes:
            raise Exception("Invalid mode: %s" % (mode, ))
        # always reset speed scalar
        self.set_speed(1.)
        # handle mode transitions
        if self.mode == 'body_restriction':
            self.res.disable()
        elif self.mode == 'leg_calibration':
            self.calibrator.detach()
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
        elif self.mode == 'leg_calibration':
            self.calibrator.attach(self.leg)
        else:
            self.all_legs('stop')

    def all_legs(self, cmd, *args, **kwargs):
        log.info({"all_legs": (cmd, args, kwargs)})
        for leg in self.legs:
            getattr(self.legs[leg], cmd)(*args, **kwargs)

    def set_leg(self, index):
        log.info({"set_leg": index})
        self.leg_index = index
        if self.leg_index is None:
            self.leg = None
            if self.mode == 'leg_calibration':
                # disable calibration
                self.calibrator.detach()
        else:
            self.leg = self.legs[index]
            if self.mode == 'leg_calibration':
                # disable, then re-enable calibration
                self.attach(self.leg)
        self.trigger('set_leg', index)

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
        elif event['name'] == 'down':
            self.set_speed(self.speed_scalar - self.speed_step)
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
                self.set_target()
            elif not event['value'] and self.deadman:
                self.all_legs('set_estop', 1)
                self.all_legs('stop')
                self.deadman = False
        elif event['name'] == 'square':
            if self.mode == 'leg_calibration':
                self.calibrator.set_subroutine('sensors', 'hip')
        elif event['name'] == 'circle':
            if self.mode == 'leg_calibration':
                self.calibrator.set_subroutine('sensors', 'thigh')
        elif event['name'] == 'cross':
            if self.mode == 'leg_calibration':
                self.calibrator.set_subroutine('sensors', 'knee')
        #elif event['name'] == 'triangle':
        #    if self.mode == 'leg_calibration':
        #        self.calibrator.set_subroutine('calf')

    def on_axis(self, event):
        # check if target vector has changed > some amount
        # if so, read and send new target
        if event['name'] in (
                'thumb_left_x', 'thumb_left_y',
                'thumb_right_x', 'thumb_right_y',
                'one_left', 'two_left'):
            if (
                    self.deadman and (
                        time.time() - self.last_joystick_move
                        > self.joystick_move_throttle)):
                self.last_joystick_move = time.time()
                print("New joystick update: %s" % self.last_joystick_move)
                self.set_target()

    def set_target(self, xyz=None):
        if xyz is None:
            ax = self.joy.axes.get('thumb_left_x', thumb_mid) - thumb_mid
            ay = self.joy.axes.get('thumb_left_y', thumb_mid) - thumb_mid
            aa = self.joy.axes.get('thumb_right_x', thumb_mid) - thumb_mid
            ab = self.joy.axes.get('thumb_right_y', thumb_mid) - thumb_mid
            az = (
                self.joy.axes.get('one_left', 0) -
                self.joy.axes.get('two_left', 0))
            if abs(ax) < thumb_db:
                ax = 0
            if abs(ay) < thumb_db:
                ay = 0
            if abs(az) < thumb_db:
                az = 0
            if abs(aa) < thumb_db:
                aa = 0
            if abs(ab < thumb_db):
                ab = 0
            #if ax == 0 and ay == 0 and az == 0:
            #    return
            ax = max(-1., min(1., ax / float(thumb_scale)))
            ay = max(-1., min(1., -ay / float(thumb_scale)))
            az = max(-1., min(1., az / 255.))
            aa = max(-1., min(1., aa / float(thumb_scale)))
            ab = max(-1., min(1., -ab / float(thumb_scale)))
            xyz = (ax, ay, az)
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
            if self.joy.keys.get('circle', 0) == 0:
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
                speed = self.speed_scalar * 0.01
                plan = {
                    'mode': consts.PLAN_ARC_MODE,
                    'frame': consts.PLAN_BODY_FRAME,
                    'linear': (0, 0, 0),
                    'angular': -numpy.array(xyz),
                    'speed': speed}
            self.all_legs('send_plan', **plan)
        elif self.mode == 'body_position_legs':
            pass
            # TODO
        elif self.mode == 'body_restriction':
            self.res.set_target(numpy.array(xyz[:2]))

    def update(self):
        if self.joy is not None:
            self.joy.update()
        if self.mode == 'leg_calibration':
            pass
        self.all_legs('update')
