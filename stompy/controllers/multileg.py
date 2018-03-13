#!/usr/bin/env python
"""
deadman: one_right [button]
thumb_left_x/y: move leg in X Y
one/two_left [axis]: move leg in Z
arrows up/down: change speed
arrows left/right: change leg (to front left/front right)

X [button]: switch to sensor coord moves
square [button]: switch to restriction
circle [button]: switch to leg coord moves
triangle [button]: move both legs

frame/mode
"""

from .. import consts
from ..leg import restriction
from .. import log


DEADMAN_KEY = 'one_right'

thumb_mid = 130
thumb_db = 5  # +-
thumb_scale = max(255 - thumb_mid, thumb_mid)


class MultiLeg(object):
    def __init__(self, legs, joy):
        self._callbacks = {}
        self.legs = legs
        self.leg = None
        self.leg_index = None
        # self.conn = leg_teensy
        self.joy = joy
        self.stopped = True
        # stop all legs
        self.all_legs('set_estop', 1)
        self.move_frame = consts.PLAN_SENSOR_FRAME
        self.speeds = {
            consts.PLAN_SENSOR_FRAME: 1200,
            consts.PLAN_LEG_FRAME: 1.5,
        }
        self.speed_scalar = 1.0
        self.speed_scalar_range = (0.1, 2.0)
        #self.res = restriction.Foot()
        #self.res.enabled = False
        #self.last_r = None

    def all_legs(self, cmd, *args):
        log.info({"all_legs": (cmd, args)})
        for leg in self.legs:
            getattr(self.legs[leg], cmd)(*args)

    def on(self, signal, func):
        if signal not in self._callbacks:
            self._callbacks[signal] = []
        self._callbacks[signal].append(func)

    def remove_on(self, signal, func):
        if signal not in self._callbacks:
            return
        if func in self._callbacks[signal]:
            self._callbacks[signal].remove(func)

    def trigger(self, signal, *args, **kwargs):
        if signal in self._callbacks:
            for f in self._callbacks[signal]:
                f(*args, **kwargs)

    def set_leg(self, index):
        log.info({"set_leg": index})
        self.leg_index = index
        if self.leg_index is None:
            self.leg = None
        else:
            self.leg = self.legs[index]
        self.trigger('set_leg', index)

    def update(self):
        evs = self.joy.update()
        # value by key name, just keep most recent
        kevs = {}
        for e in evs:
            kevs[e['name']] = e['value']
        self.all_legs('update')
        if DEADMAN_KEY in kevs:
            if kevs[DEADMAN_KEY] and self.stopped:  # pressed
                self.all_legs('set_estop', 0)
                self.all_legs('enable_pid', True)
                self.stopped = False
            elif not kevs[DEADMAN_KEY] and not self.stopped:
                # released, turn estop back on
                self.all_legs('set_estop', 1)
                self.stopped = True
        sdt = None
        if kevs.get('up', False):
            # increase speed
            sdt = 0.05
        if kevs.get('down', False):
            # decrease speed
            sdt = -0.05
        if sdt is not None:
            self.speed_scalar = max(
                self.speed_scalar_range[0],
                min(
                    self.speed_scalar_range[1],
                    self.speed_scalar + sdt))
            log.info({"speed_scalar": self.speed_scalar})
            print("Speed scalar set to: %s" % self.speed_scalar)
        new_frame = None
        if kevs.get('cross', False):
            new_frame = consts.PLAN_SENSOR_FRAME
        if kevs.get('circle', False):
            new_frame = consts.PLAN_LEG_FRAME
        #if (
        #        new_frame is not None and
        #        (new_frame != self.move_frame or self.res.enabled)):
        if (
                new_frame is not None and
                new_frame != self.move_frame):
            self.all_legs('stop')
            self.speed_scalar = 1.
            #self.res.enabled = False
            self.move_frame = new_frame
            #print("New frame: %s" % self.move_frame)
            log.info({"new_frame": new_frame})
        #  other modes...
        """
        if (
                kevs.get('square', False) and not self.res.enabled
                and not self.stopped):
            self.leg_command('stop')
            self.speed_scalar = 1.
            self.res.enabled = True
            log.info({"res_enabled": True})
            # move to swing target
            self.res.target = (
                self.conn.foot_travel_center[0],
                self.conn.foot_travel_center[1] + self.res.step_size)
            self.conn.send_plan(
                consts.PLAN_TARGET_MODE,
                consts.PLAN_LEG_FRAME,
                (
                    self.res.target[0],
                    self.res.target[1],
                    self.res.lift_height),
                speed=self.res.swing_velocity * self.speed_scalar)
            self.res.state = 'swing'
        if self.stopped:
            return
        if not self.res.enabled:
            # read joystick axes, send plan
            ax = self.joy.axes.get('thumb_left_x', thumb_mid) - thumb_mid
            ay = self.joy.axes.get('thumb_left_y', thumb_mid) - thumb_mid
            az = (
                self.joy.axes.get('one_left', 0) -
                self.joy.axes.get('two_left', 0))
            if abs(ax) < thumb_db:
                ax = 0
            if abs(ay) < thumb_db:
                ay = 0
            if abs(az) < thumb_db:
                az = 0
            if ax == 0 and ay == 0 and az == 0:
                return
            # scale to -1, 1
            ax = max(-1., min(1., ax / float(thumb_scale)))
            ay = max(-1., min(1., -ay / float(thumb_scale)))
            az = max(-1., min(1., az / 255.))
            # calculate speed
            speed = self.speeds[self.move_frame] * self.speed_scalar
            self.conn.send_plan(
                consts.PLAN_VELOCITY_MODE, self.move_frame,
                (ax, ay, az), speed=speed)
            return
        # else restriction control
        r, new_state = self.res.update(
            self.conn.xyz['x'], self.conn.xyz['y'], self.conn.xyz['z'],
            self.conn.xyz['r'], self.conn.xyz['dr'])
        dr = self.conn.xyz['dr']
        self.last_r = r
        log.debug({"r_update": (r, new_state, dr)})
        print("R: %s, dr: %s" % (r, dr))
        if new_state == 'halt':
            print("restriction too high, stopping")
            self.conn.stop()
            return
        #if self.res.state == 'stance' and r > self.res.r_thresh and dr > 0:
        #    new_state = 'lift'
        if new_state is None and sdt is not None:
            # trigger resend current plan with new speed
            new_state = self.res.state
        if new_state is not None:
            if new_state == 'swing':
                self.res.target = (
                    self.conn.foot_travel_center[0],
                    self.conn.foot_travel_center[1] + self.res.step_size)
                self.conn.send_plan(
                    consts.PLAN_TARGET_MODE,
                    consts.PLAN_LEG_FRAME,
                    (
                        self.res.target[0],
                        self.res.target[1],
                        self.res.lift_height),
                    speed=self.res.swing_velocity * self.speed_scalar)
            elif new_state == 'stance':
                self.conn.send_plan(
                    consts.PLAN_VELOCITY_MODE,
                    consts.PLAN_LEG_FRAME,
                    (0., -1., 0.),
                    speed=self.res.stance_velocity * self.speed_scalar)
            elif new_state == 'lift':
                self.conn.send_plan(
                    consts.PLAN_VELOCITY_MODE,
                    consts.PLAN_LEG_FRAME,
                    (
                        0.,
                        -self.res.stance_velocity,
                        self.res.lift_velocity),
                    speed=self.speed_scalar)
            elif new_state == 'lower':
                self.conn.send_plan(
                    consts.PLAN_VELOCITY_MODE,
                    consts.PLAN_LEG_FRAME,
                    (
                        0.,
                        -self.res.stance_velocity,
                        -self.res.lower_velocity),
                    speed=self.speed_scalar)
            print("new restriction state: %s" % new_state)
            self.res.state = new_state
        """
