#!/usr/bin/env python

import numpy


def point_on_circle(cx, cy, radius, angle):
    return (
        cx + radius * numpy.cos(angle),
        cy + radius * numpy.sin(angle),
    )


def arc(cx, cy, fx, fy, length, fr=0.5):
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
    pass
