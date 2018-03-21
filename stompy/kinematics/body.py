#!/usr/bin/env python
"""
xyz_leg = body_to_leg_transform * xyz_body

stompy is facing x+, front is x+, middle is x0, rear is x-
left is y+, right is y-
"""

import numpy
#import pylab

from .. import consts
from .. import transforms


# rotations
FLR = 145
FRR = 35
MLR = 180
MRR = 0
RLR = -145
RRR = -35

#FLR = 55
#FRR = -55
#MLR = 90
#MRR = -90
#RLR = 125
#RRR = -125

# translations
FLT = (-23, 92.6)
FRT = (23, 92.6)
MLT = (-27, 0)
MRT = (27, 0)
RLT = (-23, -92.6)
RRT = (23, -92.6)

#FLT = (92.6, 23)
#FRT = (92.6, -23)
#MLT = (0, 27)
#MRT = (0, -27)
#RLT = (-92.6, 23)
#RRT = (-92.6, -23)

leg_to_body_transforms = {
    consts.LEG_FL: transforms.affine_3d(
        FLT[0], FLT[1], 0, 0, 0, FLR, degrees=True),
    consts.LEG_FR: transforms.affine_3d(
        FRT[0], FRT[1], 0, 0, 0, FRR, degrees=True),
    consts.LEG_ML: transforms.affine_3d(
        MLT[0], MLT[1], 0, 0, 0, MLR, degrees=True),
    consts.LEG_MR: transforms.affine_3d(
        MRT[0], MRT[1], 0, 0, 0, MRR, degrees=True),
    consts.LEG_RL: transforms.affine_3d(
        RLT[0], RLT[1], 0, 0, 0, RLR, degrees=True),
    consts.LEG_RR: transforms.affine_3d(
        RRT[0], RRT[1], 0, 0, 0, RRR, degrees=True),
    consts.LEG_FAKE: transforms.affine_3d(
        0., 0., 0., 0., 0., 0., degrees=True),
}

leg_to_body_rotations = {
    consts.LEG_FL: transforms.rotation_3d(0, 0, FLR, degrees=True),
    consts.LEG_FR: transforms.rotation_3d(0, 0, FRR, degrees=True),
    consts.LEG_ML: transforms.rotation_3d(0, 0, MLR, degrees=True),
    consts.LEG_MR: transforms.rotation_3d(0, 0, MRR, degrees=True),
    consts.LEG_RL: transforms.rotation_3d(0, 0, RLR, degrees=True),
    consts.LEG_RR: transforms.rotation_3d(0, 0, RRR, degrees=True),
    consts.LEG_FAKE: transforms.rotation_3d(0, 0, 0, degrees=True),
}


body_to_leg_transforms = {
    k: numpy.linalg.inv(leg_to_body_transforms[k]) for k in
    leg_to_body_transforms}

body_to_leg_rotations = {
    k: numpy.linalg.inv(leg_to_body_rotations[k]) for k in
    leg_to_body_rotations}


def leg_to_body(leg, x, y, z):
    r = transforms.transform_3d(leg_to_body_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def leg_to_body_rotation(leg, x, y, z):
    r = transforms.transform_3d(leg_to_body_rotations[leg], x, y, z)
    return r[0], r[1], r[2]


def leg_to_body_array(leg, pts):
    return transforms.transform_3d_array(leg_to_body_transforms[leg], pts)


def leg_to_body_rotation_array(leg, pts):
    return transforms.transform_3d_array(leg_to_body_rotations[leg], pts)


def body_to_leg(leg, x, y, z):
    r = transforms.transform_3d(body_to_leg_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def body_to_leg_array(leg, pts):
    return transforms.transform_3d_array(body_to_leg_transforms[leg], pts)


def body_to_leg_rotation(leg, x, y, z):
    r = transforms.transform_3d(body_to_leg_rotations[leg], x, y, z)
    return r[0], r[1], r[2]


def body_to_leg_rotation_array(leg, pts):
    return transforms.transform_3d_array(body_to_leg_rotations[leg], pts)
