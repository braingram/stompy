#!/usr/bin/env python

import numpy


def polynomial_3(t, s, e, v_s=0., v_e=0.):
    """
    Arguments
    ===
    t : time of movement in seconds
    s : starting position
    e : ending position
    v_s : starting velocity
    v_e : ending velocity

    Returns
    ===
    function: accepts 1 argument, dt
    """
    a0 = s
    a1 = v_s
    a2 = (
        (-3 * (s - e) - (2 * v_s + v_e) * t) /
        (t * t))
    a3 = (
        (2 * (s - e) + (v_s + v_e) * t) /
        (t * t * t))
    return lambda dt: a0 + a1 * dt + a2 * dt ** 2. + a3 * dt ** 3.


def polynomial_5(t, s, e, v_s=0., v_e=0., a_s=0., a_e=0.):
    """
    Arguments
    ===
    t : time of movement in seconds
    s : starting position
    e : ending position
    v_s : starting velocity
    v_e : ending velocity
    a_s : starting acceleration
    a_e : ending acceleration

    Returns
    ===
    function: accepts 1 argument, dt
    """
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


def point_on_circle(cx, cy, radius, angle):
    return (
        cx + radius * numpy.cos(angle),
        cy + radius * numpy.sin(angle),
    )


def arc(cx, cy, fx, fy, length, fr=0.5):
    """
    Args:
        cx, cy: center of arc circle
        fx, fy: point on arc
        length: length of arc (along circle)
        fr: position of point along arc (from 0.0 to 1.0)
    Returns:
        points (function): point evaluation function that takes
            one argument, r (range [0.0, 1.0], same as fr above)
            that returns a coordinate (x, y) for a point along the arc.

    if fr = 0, the arc will start at fx, fy
    if fr = 1, the arc will end at fx, fy

    if fx, fy == cx, cy then an arc with a small (0.00001)
    radius will be returned to avoid math errors caused by
    0 radius arcs
    """
    radius = ((fx - cx) ** 2. + (fy - cy) ** 2.) ** 0.5
    if radius == 0:
        radius = 0.00001
    fa = numpy.arctan2((fy - cy), (fx - cx))
    la = length / radius

    sa = fa - la * fr

    return lambda r: point_on_circle(cx, cy, radius, r * la + sa)


def line(fx, fy, angle, length, fr=0.5):
    """
    Args:
        fx, fy: point on line
        angle: angle of line (0 = horizontal, + = counterclockwise)
        length: length of line
        fr: position of point (fx, fy) along line (from 0.0 to 1.0)
    Returns:
        points (function): point evaluation function that takes
            one argument, r (same as fr above) that returns a coordinate
            (x, y) for a point along the line

    if fr = 0.0, the line will start at fx, fy
    if fr = 1.0, the line will end at fx, fy
    """
    dx = numpy.cos(angle) * length
    dy = numpy.sin(angle) * length
    sx = fx - dx * fr
    sy = fy - dy * fr

    return lambda r: (sx + dx * r, sy + dy * r)


class Path(object):
    def __init__(self, fx, fy, easing=False):
        """
        Args:
            fx, fy: optimal foot x,y coordinate
        """
        self.fx = fx
        self.fy = fy
        self.xy = None
        # used for transitioning, skip for now
        self._next_xy = None
        if easing:
            self.set_easing()
        else:
            self.easing = lambda dt: dt

    def set_easing(self, v_s=0., v_e=0., a_s=0., a_e=0.):
        self.easing = polynomial_5(1., 0., 1., v_s, v_e, a_s, a_e)

    def arc(self, cx, cy, length, fx=None, fy=None, fr=0.5):
        if fx is None:
            fx = self.fx
        if fy is None:
            fy = self.fy
        # TODO check for 0 radius
        self.xy = arc(cx, cy, fx, fy, length, fr)

    def line(self, angle, length, fx=None, fy=None, fr=0.5):
        if fx is None:
            fx = self.fx
        if fy is None:
            fy = self.fy
        self.xy = line(fx, fy, angle, length, fr)

    def next_xy(self):
        raise NotImplementedError
        if self._next_xy is None:
            return
        f = self._next_xy[0]
        args = self._next_xy[1:]
        f(*args)
        self._next_xy = None

    def __call__(self, r):
        return self.xy(self.easing(r))
