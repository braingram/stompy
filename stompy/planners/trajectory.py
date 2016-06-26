#!/usr/bin/env python

import numpy


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
