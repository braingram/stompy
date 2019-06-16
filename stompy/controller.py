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

from . import body
from . import consts
from . import joystick
from . import kinematics
from . import leg
from . import log
from . import param
from . import playback
from . import restriction
from . import signaler


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
        #'leg_sensor',
        'leg_leg',
        'leg_body',
        #'leg_calibration',
        #'leg_restriction',
        'body',
        'sit_stand',
        'playback',
        #'body_position_legs',
        'walk',
    ]

    def __init__(self, legs, bodies):
        super(MultiLeg, self).__init__()
        self.param = param.Param()
        self.legs = legs
        self.bodies = bodies
        self.res = restriction.body.Body(legs, self.param)
        if all([isinstance(legs[ln], leg.teensy.FakeTeensy) for ln in legs]):
            # all legs are fake
            self._fake_legs = True

            # TODO make fake joystick or plan?
            # TODO start playback of fake plans?
            # allow keyboard input

            # TODO connect up odometer to fake terrain
            self._terrain = restriction.odometer.FakeTerrain()
            # connect pose callback
            
            def cbf(pose, l=self.legs, t=self._terrain):
                t.new_pose(pose, l)

            self.res.odo.on('pose', cbf)
        else:
            self._fake_legs = False
        self.leg_index = sorted(legs)[0]
        self.leg = self.legs[self.leg_index]

        self.mode = 'sit_stand'

        self.playback = None

        # if a foot gets within this distance of leg 0, throw an estop
        self.param['min_hip_distance'] = 30.0
        self.param['prevent_leg_xy_when_loaded'] = True
        self.param['min_hip_override'] = False
        self.param['speed.raw'] = 0.5
        self.param['speed.sensor'] = 1200
        self.param['speed.foot'] = 5.0
        #self.param['speed.leg'] = 6.0
        #self.param['speed.body'] = 6.0
        # assume a foot 120 inches from the rotation point to scale speeds
        self.param['arc_speed_radius'] = 120.

        #self.param['speed.body_angular'] = 0.05  # TODO convert from body

        self.param['speed.scalar'] = 1.0
        self.param.set_meta('speed.scalar', min=0.1, max=2.0, decimals=1)
        self.param['speed.step'] = 0.05

        self.height = numpy.nan
        self.joy = joystick.base.Joystick()
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
            self.param['speed.sensor'] = 4800

        # check if all legs are simulated
        if all([
                isinstance(self.legs[k], leg.teensy.FakeTeensy)
                for k in self.legs]):
            self.param['speed.foot'] = 12
            #self.param['speed.leg'] = 18
            #self.param['speed.body'] = self.param['speed.leg']
            #self.param['res.speed.stance'] = 12
            #self.param['res.speed.swing'] = 12
            #self.param['res.speed.lift'] = 12
            #self.param['res.speed.lower'] = 12

            self.set_mode('walk')
            self.param['res.max_feet_up'] = 3
            self.param['res.speed.by_restriction'] = True

    def set_speed(self, speed):
        old_speed = self.param['speed.scalar']
        meta = self.param.get_meta('speed.scalar')
        self.param['speed.scalar'] = max(
            meta['min'], min(meta['max'], speed))
        if old_speed == self.param['speed.scalar']:
            return
        log.info({"set_speed": self.param['speed.scalar']})
        self.trigger('speed', self.param['speed.scalar'])
        if self.mode == 'walk':
            self.res.set_target()

    def on_leg_estop(self, value, leg_number):
        self.trigger('estop', value)
        if value:
            for i in self.legs:
                if i != leg_number and self.legs[i].estop == consts.ESTOP_OFF:
                    self.legs[i].set_estop(value)

    def on_leg_xyz(self, xyz, leg_number):
        # TODO throttle this? pull into a structure to manage this
        # find lowest 3 legs (most negative)
        lzs = {
            l: self.legs[l].xyz.get('z', numpy.nan) for l in self.legs}
        legs_by_height = sorted(lzs.keys(), key=lambda l: lzs[l])
        self.height = -numpy.mean(sorted(lzs.values())[:3])
        # TODO limit this to only every N updates?
        self.trigger('height', self.height)
        # TODO compute support triangle, body level, COM, etc
        # look at legs in 'stance'
        # take lowest 3 feet
        if len(self.legs) < 3:
            return
        low_legs = legs_by_height[:3]
        pts = []
        for ln in low_legs:
            leg = self.legs[ln]
            # get xyz convert to body coordinates
            bxyz = kinematics.body.leg_to_body(
                ln, leg.xyz.get('x', numpy.nan),
                leg.xyz.get('y', numpy.nan),
                leg.xyz.get('z', numpy.nan))
            pts.append(bxyz)
        pts = numpy.array(pts)
        # make plane, compute pitch and roll
        # compute plane normal
        normal = numpy.cross((pts[0] - pts[1]), (pts[0] - pts[2]))
        if normal[2] < 0:
            normal *= -1.
        # theta = arctan2(o, a)
        pitch = numpy.degrees(numpy.arctan2(normal[0], normal[2]))
        roll = numpy.degrees(numpy.arctan2(normal[1], normal[2]))
        # compute COM
        return # TODO work-in-progress

    def stop(self):
        log.info({"stop": []})
        # release the deadman
        if self.deadman:
            self.all_legs('set_estop', 1)
            self.all_legs('stop')
            self.deadman = False
            self.joy._report_button('deadman', 0)

    def set_mode(self, mode):
        if mode not in self.modes:
            raise Exception("Invalid mode: %s" % (mode, ))
        # always reset speed scalar
        self.set_speed(1.)
        # handle mode transitions
        if self.mode == 'walk':
            self.res.disable()
        self.mode = mode
        self.trigger('mode', mode)
        # handle mode transitions
        if self.mode == 'walk':
            # TODO integrate this with enable
            for i in self.res.feet:
                self.res.feet[i].state = 'stance'
            self.res.enable(None)
            if (
                    self.param['res.set_height_on_mode_select'] and
                    numpy.isfinite(self.height) and
                    (self.param['res.min_lower_height'] < -self.height) and
                    (-self.height < self.param['res.max_lower_height'])):
                self.param['res.lower_height'] = -self.height
                self.trigger('config_updated')
            if self.deadman:
                self.set_target()
        elif self.mode == 'leg_pwm':
            self.all_legs('enable_pid', False)
        else:
            self.all_legs('stop')
        if self.mode == 'playback':
            # load playback filename
            #self.playback = playback.load_playback(
            #    self.param.get('playback_filename', None))
            if self._fake_legs:
                self.playback = playback.make_leg_wiggle_playback()

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
                    self.set_mode('walk')
                else:
                    self.set_mode('body')
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
            self.set_speed(
                self.param['speed.scalar'] + self.param['speed.step'])
        if buttons.get('speed_dec', 0):
            self.set_speed(
                self.param['speed.scalar'] - self.param['speed.step'])
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
        if 'restrict_leg' in buttons and self.mode == 'walk':
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
        self.param['speed.scalar'] = self.joy.axes.get('speed_axis', 0) / 255.
        self.set_speed(self.param['speed.scalar'])
        # TODO update ui

    def _set_height_by_slider(self):
        if 'height_axis' not in self.joy.axes:
            return
        h = self.joy.axes['height_axis'] / 255.
        h = (
            h * (
                self.param['res.min_lower_height'] -
                self.param['res.max_lower_height']) +
            self.param['res.max_lower_height'])
        return
        self.param['res.lower_height'] = h
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
            speed = self.param['speed.scalar'] * self.param['speed.raw']
            self.leg.set_pwm(
                xyz[0] * speed,
                xyz[1] * speed,
                xyz[2] * speed)
        #elif self.mode == 'leg_sensor':
        #    if self.leg is None:
        #        return
        #    speed = self.param['speed.scalar'] * self.param['speed.sensor']
        #    self.leg.send_plan(
        #        mode=consts.PLAN_VELOCITY_MODE,
        #        frame=consts.PLAN_SENSOR_FRAME,
        #        linear=xyz, speed=speed)
        elif self.mode == 'leg_leg':
            if self.leg is None:
                return
            #speed = self.param['speed.scalar'] * self.param['speed.leg']
            speed = self.param['speed.scalar'] * self.param['speed.foot']
            if (
                    self.param['prevent_leg_xy_when_loaded'] and
                    (self.leg.angles['calf'] >
                        self.param['res.loaded_weight'])):
                xyz = [0., 0., xyz[2]]
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_LEG_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_body':
            if self.leg is None:
                return
            #speed = self.param['speed.scalar'] * self.param['speed.body']
            speed = self.param['speed.scalar'] * self.param['speed.foot']
            if (
                    self.param['prevent_leg_xy_when_loaded'] and
                    (self.leg.angles['calf'] >
                        self.param['res.loaded_weight'])):
                xyz = [0., 0., xyz[2]]
            self.leg.send_plan(
                mode=consts.PLAN_VELOCITY_MODE,
                frame=consts.PLAN_BODY_FRAME,
                linear=xyz, speed=speed)
        elif self.mode == 'leg_calibration':
            pass
        elif self.mode == 'sit_stand':
            #speed = self.param['speed.scalar'] * self.param['speed.body']
            speed = self.param['speed.scalar'] * self.param['speed.foot']
            xyz = [0, 0, xyz[2]]
            plan = {
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_BODY_FRAME,
                'linear': -numpy.array(xyz),
                'speed': speed,
            }
            self.all_legs('send_plan', **plan)
        elif self.mode == 'body':
            if self.joy.buttons.get('sub_mode', 0) == 0:
                #speed = self.param['speed.scalar'] * self.param['speed.body']
                speed = self.param['speed.scalar'] * self.param['speed.foot']
                plan = {
                    'mode': consts.PLAN_VELOCITY_MODE,
                    'frame': consts.PLAN_BODY_FRAME,
                    'linear': -numpy.array(xyz),
                    'speed': speed,
                }
            else:
                # swap x and y
                xyz = [xyz[1], xyz[0], xyz[2]]
                #speed = (
                #    self.param['speed.scalar'] *
                #    self.param['speed.body_angular'])
                speed = (
                    self.param['speed.scalar'] *
                    self.param['speed.foot'] / self.param['arc_speed_radius'])
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
        elif self.mode == 'walk':
            # pass in rx, ly, az
            # also pass in mode for crab walking
            omni_walk = bool(self.joy.buttons.get('sub_mode', 0))
            # convert joystick input to a rotation about some point
            # need to calculate
            # - center of rotation (in body coordinates)
            # - linear speed during rotation
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
            # add dz
            # TODO maybe this should be scaled down?
            dz = -xyz[2] * self.param['speed.foot'] * self.param['speed.scalar'] * consts.PLAN_TICK
            if self.param['res.speed.by_restriction']:
                dz *= self.res.get_speed_by_restriction()
            self.res.set_target(
                restriction.body.BodyTarget((crx, cry), rs, dz))

    def update(self):
        self.joy.update()
        self.all_legs('update')
        if self.mode == 'playback' and self.playback is not None:
            self.playback.update(self)
        elif self.mode in ('body', 'walk', 'playback', 'sit_stand'):
            if self.param['min_hip_override']:
                # check if override should be turned off
                disable_override = True
                threshold = self.param['min_hip_distance'] * 1.5
                for l in self.legs:
                    if self.legs[l].xyz.get('x', 0) < threshold:
                        disable_override = False
                if disable_override:
                    self.param['min_hip_override'] = False
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
                    if (
                            self.legs[l].xyz.get('x', 0) <
                            self.param['min_hip_distance']):
                        # set estop
                        trigger_estop = True
                # one of the legs is too close to the hip and not all
                # are stopped so trigger an estop on all legs
                if not all_stopped and trigger_estop:
                    print("estopping because foot too close to hip")
                    self.all_legs('set_estop', consts.ESTOP_DEFAULT)
        # update all body teensies
        [self.bodies[k].update() for k in self.bodies]


def build():
    #if joystick.ps3.available():
    #    joy = joystick.ps3.PS3Joystick()
    #elif joystick.steel.available():
    #    joy = joystick.steel.SteelJoystick()
    #else:
    #    joy = joystick.fake.FakeJoystick()

    legs = leg.teensy.connect_to_teensies()

    if len(legs) == 0:
        raise IOError("No teensies found")

    lns = sorted(legs.keys())
    print("Connected to legs: %s" % (lns, ))

    bodies = body.connect_to_teensies()
    print("Connected to bodies: %s" % (sorted(bodies.keys())))

    return MultiLeg(legs, bodies)


def run(controller=None):
    if controller is None:
        controller = build()
    print("Built controller, running in loop [Ctrl-C to exit]")
    while True:
        try:
            controller.update()
            time.sleep(0.01)
        except KeyboardInterrupt:
            break
