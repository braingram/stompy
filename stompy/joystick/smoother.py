#!/usr/bin/env python

import time


def polynomial_5(s, e, v_s, v_e, a_s, a_e, t):
    a0 = s
    a1 = v_s
    a2 = a_s * 0.5
    a3 = (
        (20 * (e - s) - (8 * v_e + 12 * v_s) * t - (3 * a_e - a_s) * t ** 2.) /
        (2. * t ** 3.))
    a4 = (
        (
            -30 * (e - s) + (14 * v_e + 16 * v_s) * t +
            (3 * a_e - 2 * a_s) * t ** 2.) /
        (2. * t ** 4.))
    a5 = (
        (12 * (e - s) - 6 * (v_e + v_s) * t + (a_e - a_s) * t ** 2.) /
        (2 * t ** 5.))
    return lambda dt: (
        a0 + a1 * dt + a2 * dt ** 2. + a3 * dt ** 3. +
        a4 * dt ** 4. + a5 * dt ** 5.)


class AxisSmoother(object):
    def __init__(self, settle_time):
        self._last_setpoint = 0.
        self.pos = 0.
        self.vel = 0.
        self._dt = 0.
        self._last_update = time.time()
        self._f = None
        self.settle_time = settle_time

    def reset(self, pos=None):
        if pos is not None:
            self.pos = pos
        self.vel = 0.
        self._f = None
        self._last_update = time.time()
        self._dt = 0.
        self._last_setpoint = pos

    def update(self, setpoint=None, t=None):
        if t is None:
            t = time.time()
        dt = (t - self._last_update)
        self._last_update = t
        if setpoint is not None:
            if (abs(self._last_setpoint - setpoint) > 0.001):
                #self.vel = 0.
                self._f = polynomial_5(
                    self.pos, setpoint, self.vel, 0., 0., 0., self.settle_time)
                self._last_setpoint = setpoint
                self._dt = 0.
        if dt == 0:
            print("time same, not updating")
            return self.pos
        self._dt += dt
        if self._f is None or self._dt >= self.settle_time:
            self._f = None
            np = self.pos
        else:
            np = self._f(self._dt)
        op = self.pos
        self.pos = np
        self.vel = (self.pos - op) / dt
        return self.pos
