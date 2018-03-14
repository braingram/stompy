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


def point_to_angles(x, y, z):
    HL = 11.
    KL = 72.
    TL = 54.
    TRA = 1.4663392033212757
    BB = 0.22670942831590035
    l = (x * x + y * y) ** 0.5
    hip = numpy.arctan2(y, x)
    L = (z * z + (l - HL) * (l - HL)) ** 0.5
    a1 = numpy.arccos(-z / L)
    a2 = numpy.arccos(
        (KL * KL - TL * TL - L * L) /
        (-2 * TL * L))
    alpha = (a1 + a2)
    beta = numpy.arccos(
        (L * L - KL * KL - TL * TL) /
        (-2 * KL * TL))
    thigh = TRA - (alpha - numpy.pi / 2.)
    knee = BB - beta
    return hip, thigh, knee
