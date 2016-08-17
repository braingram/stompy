#!/usr/bin/env python

import numpy


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

    def point(r):
        return point_on_circle(cx, cy, radius, r * la + sa)
    return point


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
    print dx, dy, sx, sy

    def point(r):
        return sx + dx * r, sy + dy * r
    return point


class Path(object):
    def __init__(self, fx, fy):
        """
        Args:
            fx, fy: optimal foot x,y coordinate
        """
        self.fx = fx
        self.fy = fy
        self.xy = None
        self._next_xy = None

    def arc(self, cx, cy, length, fx=None, fy=None, fr=None):
        if fr is not None:
            assert fx is not None
            assert fy is not None
            self._next_xy = (self.arc, cx, cy, length)
        else:
            fr = 0.5
            fx = self.fx
            fy = self.fy
            self._next_xy = None
        self.xy = arc(cx, cy, fx, fy, length, fr)

    def line(self, angle, length, fx=None, fy=None, fr=None):
        if fr is not None:
            assert fx is not None
            assert fy is not None
            self._next_xy = (self.line, angle, length)
        else:
            fr = 0.5
            fx = self.fx
            fy = self.fy
            self._next_xy = None
        self.xy = line(fx, fy, angle, length, fr)

    def next_xy(self):
        if self._next_xy is None:
            return
        f = self._next_xy[0]
        args = self._next_xy[1:]
        f(*args)
        self._next_xy = None

    def __call__(self, r):
        return self.xy(r)
