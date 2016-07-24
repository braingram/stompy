#!/usr/bin/env python
"""
xyz_leg = body_to_leg_transform * xyz_body

stompy is facing x+, front is x+, middle is x0, rear is x-
left is y+, right is y-
"""

import numpy
import pylab

from .. import transforms


leg_to_body_transforms = {
    'fl': transforms.affine_3d(
        2.235, 0.584, 0, 0, 0, 55, degrees=True),
    'fr': transforms.affine_3d(
        2.235, -0.584, 0, 0, 0, -55, degrees=True),
    'ml': transforms.affine_3d(
        0., 0.686, 0, 0, 0, 90, degrees=True),
    'mr': transforms.affine_3d(
        0., -0.686, 0, 0, 0, -90, degrees=True),
    'rl': transforms.affine_3d(
        -2.235, 0.584, 0, 0, 0, 125, degrees=True),
    'rr': transforms.affine_3d(
        -2.235, -0.584, 0, 0, 0, -125, degrees=True),
    #'fl': transforms.affine_2d(2.235, 0.584, 55, degrees=True),
    #'fr': transforms.affine_2d(2.235, -0.584, -55, degrees=True),
    #'ml': transforms.affine_2d(0., 0.686, 90, degrees=True),
    #'mr': transforms.affine_2d(0., -0.686, -90, degrees=True),
    #'rl': transforms.affine_2d(-2.235, 0.584, 125, degrees=True),
    #'rr': transforms.affine_2d(-2.235, -0.584, -125, degrees=True),
}


body_to_leg_transforms = {
    k: numpy.linalg.inv(leg_to_body_transforms[k]) for k in
    leg_to_body_transforms}


def leg_to_body(leg, x, y, z):
    r = transforms.transform_3d(leg_to_body_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def body_to_leg(leg, x, y, z):
    r = transforms.transform_3d(body_to_leg_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def plot_legs():
    p0 = (0., 0., 0.)
    p1 = (1., 0., 0.)
    for leg in leg_to_body_transforms:
        tp0 = leg_to_body(leg, *p0)
        tp1 = leg_to_body(leg, *p1)
        pylab.plot([tp0[0], tp1[0]], [tp0[1], tp1[1]], label=leg)
    pylab.legend()
