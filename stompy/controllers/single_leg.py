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
        if self.stopped:
            return
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
        ay = max(-1., min(1., ay / float(thumb_scale)))
        az = max(-1., min(1., az / 255.))
        # calculate speed
        speed = self.speeds[self.move_frame] * self.speed_scalar
        # else send plan
        self.conn.send_plan(
            consts.PLAN_VELOCITY_MODE, self.move_frame,
            (ax, ay, az), speed=speed)
