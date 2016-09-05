#!/usr/bin/env python

import numpy

from .. import transforms


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


def linear_by_n(start, end, n):
    start = numpy.array(start)
    end = numpy.array(end)
    return numpy.vstack(
        [numpy.linspace(s, e, n) for (s, e) in zip(start, end)]
    ).T


def linear_by_distance(start, end, distance):
    start = numpy.array(start)
    end = numpy.array(end)
    n = numpy.ceil(numpy.linalg.norm(end - start) / float(distance))
    pts = linear_by_n(start, end, n)
    if numpy.sum(numpy.abs(pts[-1] - end)) > 0.0001:
        return numpy.vstack((pts, end))
    return pts


def linear_by_rate(start, end, time, rate):
    return linear_by_n(start, end, time * rate)


def follow_transform(start, transform, n_steps):
    """returns n_steps + 1 points"""
    pts = []
    pt = start
    pts.append(pt)
    for i in xrange(n_steps):
        pt = transforms.transform_3d(transform, *pt)
        pts.append(pt)
    return pts


class PathTracer(object):
    def __init__(self, start=None, end=None, time=None, rate=None):
        self.start = start
        self.end = end
        self.time = time
        self.rate = rate
        self.i = None
        self.n = None

    def setup(self):
        self.start = numpy.array(self.start)
        self.end = numpy.array(self.end)
        self.n = self.time * self.rate
        self.delta = (self.end - self.start) / float(self.n - 1)
        self.i = 1

    def __iter__(self):
        return self

    def next(self):
        if self.i is None:
            self.setup()
        if self.i < self.n:
            i = self.i
            self.i += 1
            return self.start + self.delta * i
        else:
            return None


class DirectionTracer(object):
    def __init__(self, start=None, delta=None):
        self.start = start
        self.delta = delta

    def setup(self):
        self.start = numpy.array(self.start)
        self.delta = numpy.array(self.delta)

    def __iter__(self):
        return self

    def next(self):
        self.start += self.delta
        return self.start


class Trajectory(object):
    def __init__(self, points, times):
        pass

    def update(self, time):
        # update tracking current trajectory
        pass
