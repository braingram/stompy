#!/usr/bin/env python

import numpy

from .. import transforms


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
