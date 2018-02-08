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
        self.conn.ns.estop(1)
        self.move_frame = consts.PLAN_SENSOR_FRAME
        self.speeds = {
            consts.PLAN_SENSOR_FRAME: 1500,
            consts.PLAN_LEG_FRAME: 10.0,
        }
        self.speed_scalar = 1.0
        self.speed_scalar_range = (0.1, 2.0)
        self.res = restriction.Foot()
        self.res.enabled = False

    def run_threads(self):
        self.joy.start_update_thread()
        self.conn.start_update_thread()
        self._threads_running = True

    def update(self):
        if not self._threads_running:
            self.joy.update()
            self.conn.update()
        if DEADMAN_KEY in self.joy.key_edges:
            e = self.joy.key_edges[DEADMAN_KEY]
            if e['value']:  # pressed
                self.conn.ns.estop(0)
                self.conn.ns.enable_pid(True)
                self.stopped = False
            else:  # released, turn estop back on
                self.conn.ns.estop(1)
                self.stopped = True
            self.joy.clear_key_edge(DEADMAN_KEY)
        sdt = None
        if 'up' in self.joy.key_edges:
            # increase speed
            if self.joy.key_edges['up']['value']:
                sdt = 0.1
            self.joy.clear_key_edge('up')
        if 'down' in self.joy.key_edges:
            # decrease speed
            if self.joy.key_edges['down']['value']:
                sdt = -0.1
            self.joy.clear_key_edge('down')
        if sdt is not None:
            self.speed_scalar = max(
                self.speed_scalar_range[0],
                min(
                    self.speed_scalar_range[1],
                    self.speed_scalar + sdt))
            print("Speed scalar set to: %s" % self.speed_scalar)
        new_frame = None
        if 'cross' in self.joy.key_edges:
            if self.joy.key_edges['cross']['value']:
                new_frame = consts.PLAN_SENSOR_FRAME
            self.joy.clear_key_edge('cross')
        if 'circle' in self.joy.key_edges:
            if self.joy.key_edges['circle']['value']:
                new_frame = consts.PLAN_LEG_FRAME
            self.joy.clear_key_edge('circle')
        if new_frame is not None:
            self.conn.stop()
            self.speed_scalar = 1.
            self.res.enabled = False
            self.move_frame = new_frame
            print("New frame: %s" % self.move_frame)
        if 'square' in self.joy.key_edges:
            if self.joy.key_edges['square']['value']:
                self.conn.stop()
                self.speed_scalar = 1.
                self.res.enabled = True
            self.joy.clear_key_edge('square')
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
            self.conn.xyz['x'], self.conn.xyz['y'], self.conn.xyz['z'])
        if new_state == 'halt':
            print("restriction too high, stopping")
            self.conn.stop()
            return
        if self.res.state == 'stance' and r > self.res.r_thresh:
            new_state = 'lift'
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
