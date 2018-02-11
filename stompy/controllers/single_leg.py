#!/usr/bin/env python
"""
single leg controller:
    - read joystick & teensy
    - handle joystick events
    - change modes:
        - raw pwm
        - sensor coord moves
        - joint coord moves
        - leg coord moves
        - body coord moves
        - restriction control
    - manage speeds
    - manage logging
    - present data for UI (?)


deadman: one_right [button]
    when pressed: disable estop
    when released: enable estop, enable_pids (always?)
thumb_left_x/y: move leg in X Y
one/two_left [axis]: move leg in Z
X [button]: switch to sensor coord moves
square [button]: switch to restriction
circle [button]: switch to leg coord moves
triangle [button]: ?
d-pad: increase, decrease speed [within bound]
"""

from ..leg import consts
from ..leg import restriction
from .. import log


DEADMAN_KEY = 'one_right'

thumb_mid = 130
thumb_db = 5  # +-
thumb_scale = max(255 - thumb_mid, thumb_mid)


class SingleLeg(object):
    def __init__(self, leg_teensy, joy):
        self.conn = leg_teensy
        self.joy = joy
        self._threads_running = False
        self.stopped = True
        self.conn.set_estop(1)
        self.move_frame = consts.PLAN_SENSOR_FRAME
        self.speeds = {
            consts.PLAN_SENSOR_FRAME: 1200,
            consts.PLAN_LEG_FRAME: 5.0,
        }
        self.speed_scalar = 1.0
        self.speed_scalar_range = (0.1, 2.0)
        self.res = restriction.Foot()
        self.res.enabled = False
        self.last_r = None

    def update(self):
        evs = self.joy.update()
        # value by key name, just keep most recent
        kevs = {}
        for e in evs:
            kevs[e['name']] = e['value']
        self.conn.update()
        if DEADMAN_KEY in kevs:
            if kevs[DEADMAN_KEY] and self.stopped:  # pressed
                self.conn.set_estop(0)
                self.conn.enable_pid(True)
                self.stopped = False
            elif not kevs[DEADMAN_KEY] and not self.stopped:
                # released, turn estop back on
                self.conn.set_estop(1)
                self.stopped = True
        sdt = None
        if kevs.get('up', False):
            # increase speed
            sdt = 0.1
        if kevs.get('down', False):
            # decrease speed
            sdt = -0.1
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
        if (
                new_frame is not None and
                (new_frame != self.move_frame or self.res.enabled)):
            self.conn.stop()
            self.speed_scalar = 1.
            self.res.enabled = False
            self.move_frame = new_frame
            print("New frame: %s" % self.move_frame)
            log.info({"new_frame": new_frame})
        if (
                kevs.get('square', False) and not self.res.enabled
                and not self.stopped):
            self.conn.stop()
            self.speed_scalar = 1.
            self.res.enabled = True
            log.info({"res_enabled": True})
            # move to swing target
            self.res.target = (
                self.res.center[0],
                self.res.center[1] + self.res.step_size)
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
        if new_state is not None:
            if new_state == 'swing':
                self.res.target = (
                    self.res.center[0],
                    self.res.center[1] + self.res.step_size)
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
