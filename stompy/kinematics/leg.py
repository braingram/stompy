#!/usr/bin/env python

import numpy


HIP_LENGTH = 11
THIGH_LENGTH = 54
KNEE_LENGTH = 72
THIGH_REST_ANGLE = 1.4663392033212757
KNEE_REST_ANGLE = -1.4485440219526171


def angles_to_points(hip, thigh, knee):
    x = HIP_LENGTH
    z = 0
    ch = numpy.cos(hip)
    sh = numpy.sin(hip)
    yield x * ch, x * sh, z

    a = THIGH_REST_ANGLE - thigh
    x += THIGH_LENGTH * numpy.cos(a)
    z += THIGH_LENGTH * numpy.sin(a)
    yield x * ch, x * sh, z

    a = KNEE_REST_ANGLE - knee - thigh
    x += KNEE_LENGTH * numpy.cos(a)
    z += KNEE_LENGTH * numpy.sin(a)

    yield x * ch, x * sh, z
