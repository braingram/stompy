#!/usr/bin/env python
"""
xyz_leg = body_to_leg_transform * xyz_body

stompy is facing x+, front is x+, middle is x0, rear is x-
left is y+, right is y-
"""

import numpy
import pylab


def translation_matrix(x, y):
    return numpy.matrix(
        [[1., 0., x], [0., 1., y], [0., 0., 1.]])


def rotation_matrix(a, degrees=False):
    if degrees:
        a = numpy.radians(a)
    sa = numpy.sin(a)
    ca = numpy.cos(a)
    return numpy.matrix(
        [[ca, -sa, 0.], [sa, ca, 0.], [0., 0., 1.]])


def affine_matrix(x, y, a, degrees=False):
    #return rotation_matrix(a, degrees) * translation_matrix(x, y)
    return translation_matrix(x, y) * rotation_matrix(a, degrees)


def transform(x, y, m):
    r = m * [[x], [y], [1.]]
    return float(r[0]), float(r[1])


leg_to_body_transforms = {
    'fl': affine_matrix(2.235, 0.584, 55, degrees=True),
    'fr': affine_matrix(2.235, -0.584, -55, degrees=True),
    'ml': affine_matrix(0., 0.686, 90, degrees=True),
    'mr': affine_matrix(0., -0.686, -90, degrees=True),
    'rl': affine_matrix(-2.235, 0.584, 125, degrees=True),
    'rr': affine_matrix(-2.235, -0.584, -125, degrees=True),
}


body_to_leg_transforms = {
    k: numpy.linalg.inv(leg_to_body_transforms[k]) for k in
    leg_to_body_transforms}


def leg_to_body(leg, x, y, z):
    r = transform(x, y, leg_to_body_transforms[leg])
    return r[0], r[1], z


def body_to_leg(leg, x, y, z):
    r = transform(x, y, body_to_leg_transforms[leg])
    return r[0], r[1], z


def plot_legs():
    p0 = (0., 0., 0.)
    p1 = (1., 0., 0.)
    for leg in leg_to_body_transforms:
        tp0 = leg_to_body(leg, *p0)
        tp1 = leg_to_body(leg, *p1)
        pylab.plot([tp0[0], tp1[0]], [tp0[1], tp1[1]], label=leg)
    pylab.legend()
