#!/usr/bin/env python
"""
Per-leg joint geometry and pre-computer

Each joint is defined by a triangle with three edges:
    a: short static side
    b: long static side
    c: cylinder
with corresponding angles (C, angle opposite c, etc).
In all cases, the angle of interest for the joint is C (opposite the cylinder)

Takes in custom joint geometry (as measured on a leg):
    - cylinder min, cylinder max (range of triangle edge c)
    - triangle edge a (short static side)
    - triangle edge b (long static side)
    - joint lengths
    - rest angles (angle offset when joint angles are 0)

and computes:
    - joint limits: min and max angles
    - zero angles (hip: cylinders equal, thigh: retracted, knee: extended)

when all joints are computed, also precompute some values for kinematics:
    - base beta
"""

import copy

import numpy


default_geometry = {
    'hip': {
        'link': {
            'length': 11.0,
            'angle': 0.,  # rest angle
        },
        'cylinder': {
            'min': 16.0,  # TODO measure this!
            'max': 23.153837,  # TODO measure this!
            'zero_angle_length': 19.8173,  # TODO measure this!
        },
        'triangle': {
            'a': 6.83905,
            'b': 19.16327,
        },
    },
    'thigh': {
        'link': {
            'length': 54.0,
            'angle': numpy.radians(84.0),  # rest angle
        },
        'cylinder': {
            'min': 24.0,
            'max': 38.0,
        },
        'triangle': {
            'a': 10.21631,
            'b': 33.43093,
        },
    },
    'knee': {
        'link': {
            'length': 72.0,
            'angle': numpy.radians(-83.0),  # rest angle
        },
        'cylinder': {
            'min': 20.0,
            'max': 32.0,
        },
        'triangle': {
            'a': 7.4386,
            'b': 25.6021,
        },
    },
}

# if not in here, use defaults
per_leg_geometry = {
    2: {
        'hip': {
            'cylinder': {
                'max': 24.0,  # TODO measure this!
                'zero_angle_length': 20.30,  # TODO measure this!
            },
            'triangle': {
                'b': 19.62051,  # TODO measure this!
            },
        },
    },
    5: {
        'hip': {
            'cylinder': {
                'max': 24.0,
                'zero_angle_length': 20.30,
            },
            'triangle': {
                'b': 19.62051,
            },
        },
    },
}


def merge_dicts(bd, md):
    for k in md:
        if k in bd and isinstance(md[k], dict):
            bd[k] = merge_dicts(bd[k], md[k])
        else:
            bd[k] = copy.deepcopy(md[k])
    return bd


def abc_to_C(a, b, c):
    # cosine rule, given 3 sides (where c is cylinder)
    # compute angle C (joint angle)
    return numpy.arccos((a * a + b * b - c * c) / (2 * a * b))


def add_zero_angle_lengths(leg_geometry):
    """add cylinder zero angle lengths to
    [joint]['cylinder']['zero_angle_length']"""
    # 'hip' use hip zero length
    if 'zero_angle_length' not in leg_geometry['hip']['cylinder']:
        raise ValueError("Hip cylinder zero angle length must be measured")

    # 'thigh': cylinder at min
    if 'zero_angle_length' not in leg_geometry['thigh']['cylinder']:
        leg_geometry['thigh']['cylinder']['zero_angle_length'] = \
            leg_geometry['thigh']['cylinder']['min']

    # 'knee': cylinder at max
    if 'zero_angle_length' not in leg_geometry['knee']['cylinder']:
        leg_geometry['knee']['cylinder']['zero_angle_length'] = \
            leg_geometry['knee']['cylinder']['max']
    return leg_geometry


def add_zero_angles(leg_geometry):
    """add zero angles to [joint]['link']['zero_angle']"""
    for jn in ('hip', 'thigh', 'knee'):
        jg = leg_geometry[jn]
        jt = jg['triangle']
        c = jg['cylinder']['zero_angle_length']
        jg['link']['zero_angle'] = abc_to_C(jt['a'], jt['b'], c)
    return leg_geometry


def add_joint_limits(leg_geometry):
    for jn in ('hip', 'thigh', 'knee'):
        jg = leg_geometry[jn]
        jt = jg['triangle']

        # compute angle at cylinder min
        c = jg['cylinder']['min']
        min_angle = abc_to_C(jt['a'], jt['b'], c) - jg['link']['zero_angle']

        # compute angle at cylinder max
        c = jg['cylinder']['max']
        max_angle = abc_to_C(jt['a'], jt['b'], c) - jg['link']['zero_angle']

        # if this is the hip, there is an opposing cylinder
        # so take shortest excursion
        if jn == 'hip':
            max_angle = min(abs(min_angle), abs(max_angle))
            min_angle = -max_angle
        jg['limits'] = {
            'min': min_angle,
            'max': max_angle,
        }
    return leg_geometry


def add_kinematic_constants(leg_geometry):
    # base beta = pi - thigh_rest - knee_rest, see inverse kinematics
    leg_geometry['base_beta'] = (
        leg_geometry['thigh']['link']['angle'] -
        leg_geometry['knee']['link']['angle'])
    return leg_geometry


def pre_compute(leg_geometry):
    leg_geometry = add_zero_angle_lengths(leg_geometry)
    leg_geometry = add_zero_angles(leg_geometry)
    leg_geometry = add_joint_limits(leg_geometry)
    leg_geometry = add_kinematic_constants(leg_geometry)
    return leg_geometry


def get_leg_geometry(leg_number):
    # get base leg geometry
    leg_geometry = copy.deepcopy(default_geometry)

    # combine with any per-leg measurements
    if leg_number in per_leg_geometry:
        leg_geometry = merge_dicts(leg_geometry, per_leg_geometry[leg_number])

    # pre-compute
    return pre_compute(leg_geometry)
