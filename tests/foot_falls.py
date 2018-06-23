#!/usr/bin/env python
"""
given a target (arc, possible z translation)
and a z height

find the optimal foot fall that is:
    - within some % or distance of edge?
    - at least distance from hip center?


target is:
    - rotation about point (speed doesn't matter)
    - possible z translation
"""

import sys
import time

import matplotlib.patches
import numpy
import pylab

import stompy


# target
tx, ty = 50, 0  # gives 0 ipts
tx, ty = 40, 0  # gives > 2 ipts
tx, ty = 40, 40
#tx, ty = -100000, 0
rspeed = 0.005
tdz = 0.

z = -75
leg_number = 7

# min distance from leg 0,0
min_hip_distance = None

step_ratio = 0.8

"""
def center_at_z(z):
    # find center position at a given z height
    # hip = 0, y = 0
    # get limits at z
    # find mid point
    l, r = stompy.kinematics.leg.limits_at_z_2d(z)
    return ((l + r) / 2., 0.)


def x_with_vertical_calf(z):
    # thigh rest angle is 1.466 (84 degrees)
    # increasing thigh angle decreases this
    # knee is 90 - thigh rest angle
    # knee rest angle is -1.44 (-82 degrees) [I think this is abs]

    # kra - k - t = pi / 2

    # z = tl * sin (tra - t) - kl
    # tra - arcsin((z + kl) / tl) = t
    # t = tra - arcsin((z + kl) / tl)
    # k = kra - t - pi / 2

    #t = (
    #    stompy.geometry.THIGH_REST_ANGLE -
    #    numpy.arcsin((
    #        z + stompy.geometry.KNEE_LENGTH) / stompy.geometry.THIGH_LENGTH))
    #k = stompy.geometry.KNEE_REST_ANGLE - t - numpy.pi / 2.
    # simplify this to just compute x
    #x, _, _ = list(stompy.kinematics.leg.angles_to_points(0, t, k))[-1]
    #x = (
    #    stompy.geometry.HIP_LENGTH +
    #    stompy.geometry.THIGH_LENGTH * numpy.cos(
    #        stompy.geometry.THIGH_REST_ANGLE - t))
    #x = (
    #    stompy.geometry.HIP_LENGTH +
    #    stompy.geometry.THIGH_LENGTH * numpy.cos(
    #        numpy.arcsin((
    #            z + stompy.geometry.KNEE_LENGTH) /
    #            stompy.geometry.THIGH_LENGTH)))
    return (
        stompy.geometry.HIP_LENGTH +
        stompy.geometry.THIGH_LENGTH *
        numpy.sqrt(1 - ((
            z + stompy.geometry.KNEE_LENGTH) /
            stompy.geometry.THIGH_LENGTH) ** 2.))
"""

"""
def target_circle(tx, ty, fx, fy):
    # circle center is tx, ty
    # radius is distance between tx, ty and fx, fy
    return {
        'center': (tx, ty),
        'radius': numpy.sqrt((tx - fx) ** 2. + (ty - fy) ** 2.),
    }
"""
"""
def find_limit_intersections(c, z, leg_number, min_hip_distance=None):
    # get 'left' [closest to hip] and 'right' circles
    # get hip limits (sets +-y angle)
    # TODO account for z translation
    l, r = stompy.kinematics.leg.limits_at_z_2d(z)
    # if l is too close to hip, extend it out
    if min_hip_distance is not None:
        l = max(l, min_hip_distance)
    lc = {'center': (0, 0), 'radius': l}
    rc = {'center': (0, 0), 'radius': r}
    # intersection with hip lines
    if leg_number in stompy.consts.MIDDLE_LEGS:
        hmin, hmax = (
            stompy.geometry.HIP_MIDDLE_MIN_ANGLE,
            stompy.geometry.HIP_MIDDLE_MAX_ANGLE)
    else:
        hmin, hmax = (
            stompy.geometry.HIP_MIN_ANGLE,
            stompy.geometry.HIP_MAX_ANGLE)

    camin = numpy.cos(hmin)
    samin = numpy.sin(hmin)
    camax = numpy.cos(hmax)
    samax = numpy.sin(hmax)
    min_lix, min_liy = camin * l, samin * l
    min_rix, min_riy = camin * r, samin * r
    max_lix, max_liy = camax * l, samax * l
    max_rix, max_riy = camax * r, samax * r
    min_ci = stompy.kinematics.leg.circle_intersection(c, lc)
    max_ci = stompy.kinematics.leg.circle_intersection(c, rc)
    min_li0, min_li1 = stompy.kinematics.leg.circle_line_segment_intersection(
        c, [min_lix, min_liy], [min_rix, min_riy])
    max_li0, max_li1 = stompy.kinematics.leg.circle_line_segment_intersection(
        c, [max_lix, max_liy], [max_rix, max_riy])
    ipts = []
    for v in (min_li0, min_li1, max_li0, max_li1):
        if v is not None:
            ipts.append(v)
    for ci in (min_ci, max_ci):
        if ci is not None:
            for p in ci:
                if p is None:
                    continue
                x, y = p
                # calculate angle from 0, 0 to x, y
                # if angle is withing min/max, keep
                # tan(theta) = y / x
                a = numpy.arctan2(y, x)
                if hmin <= a <= hmax:
                    ipts.append(p)
    return ipts
    return (
        stompy.kinematics.leg.circle_intersection(c, lc),
        stompy.kinematics.leg.circle_intersection(c, rc),
        stompy.kinematics.leg.circle_line_segment_intersection(
            c, [camin * l, samin * l], [camin * r, samin * r]),
        stompy.kinematics.leg.circle_line_segment_intersection(
            c, [camax * l, samax * l], [camax * r, samax * r]),
    )
"""


def swing_position_from_intersections(tc, rspeed, c0, ipts, step_ratio):
    # if len(ipts) == 0, return to center?
    if len(ipts) == 0:
        return c0
    # if rspeed > 0: rotating clockwise, find point counterclockwise to 0
    # if rspeed < 0: rotating counterclockwise, find point clockwise to 0
    tc = numpy.array(tc)
    cv = numpy.array(c0) - tc
    cvn = cv / numpy.linalg.norm(cv)
    ma = None
    mi = None
    mas = None
    for i in ipts:
        iv = numpy.array(i) - tc
        ivn = iv / numpy.linalg.norm(iv)
        #a = numpy.arctan2(
        #    numpy.linalg.norm(numpy.cross(iv, cv)), numpy.dot(iv, cv))
        angle_sign = numpy.sign(numpy.cross(ivn, cvn))
        if numpy.sign(rspeed) != angle_sign:
            continue
        a = numpy.arccos(numpy.clip(numpy.dot(ivn, cvn), -1.0, 1.0))
        if ma is None or a < ma:
            ma = a
            mi = i
            mas = angle_sign
        #a = numpy.arccos(
        #    numpy.dot(iv, cv) /
        #    (numpy.linalg.norm(iv) * numpy.linalg.norm(cv)))
        print(i, a)
    if ma is None:
        return c0
    #return mi
    pa = -mas * ma * step_ratio
    # rotate vector from tc to c0 (cv) by angle pa
    ca = numpy.cos(pa)
    sa = numpy.sin(pa)
    x, y = cv
    return (
        x * ca - y * sa + tc[0],
        x * sa + y * ca + tc[1])


def calculate_swing_target(
        tx, ty, z, leg_number, rspeed, step_ratio,
        min_hip_distance=None,
        plot=True):
    t0 = time.time()
    # get x with vertical calf
    # vertical calf doesn't work with z > -19 or z < -75,
    # I don't think we can walk with
    # legs this high/low anyway
    # TODO cache these, they're used >1 time
    l, r = stompy.kinematics.leg.limits_at_z_2d(z)
    c0x = stompy.kinematics.leg.x_with_vertical_calf(z)
    if c0x <= l or c0x >= r:
        c0x, _ = stompy.kinematics.leg.xy_center_at_z(z)
    # calculate target movement circle using center of tx, ty
    #tc = target_circle(tx, ty, c0x, 0.)
    tc = {
        'center': (tx, ty),
        'radius': numpy.sqrt((tx - c0x) ** 2. + (ty - 0.) ** 2.),
    }
    ipts = stompy.kinematics.leg.find_limit_intersections(
        tc, z, leg_number, min_hip_distance=min_hip_distance)
    sp = swing_position_from_intersections(
        [tx, ty], rspeed, [c0x, 0], ipts, step_ratio)
    t1 = time.time()
    print("computing swing took: %s" % (t1 - t0))
    if plot:
        lpts = numpy.array(
            stompy.kinematics.leg.limits_at_z_3d(z, leg_number))
        pylab.plot(lpts[:, 0], lpts[:, 1])
        #pylab.scatter(x, y, color='b')
        pylab.scatter(c0x, 0., color='r')
        pylab.gca().set_aspect(1.0)
        c = matplotlib.patches.Circle(
            tc['center'], tc['radius'], facecolor='none', edgecolor='g')
        pylab.gca().add_artist(c)

        if len(ipts):
            ipts = numpy.array(ipts)
            pylab.scatter(ipts[:, 0], ipts[:, 1], color='m')
        pylab.scatter(sp[0], sp[1], color='k', s=100)
        pylab.show()
    return sp


def test():
    t0 = time.time()
    if len(sys.argv) > 1:
        z = float(sys.argv[1])
    print("At z: %s" % z)
    lpts = numpy.array(stompy.kinematics.leg.limits_at_z_3d(z, leg_number))
    x, y = stompy.kinematics.leg.xy_center_at_z(z)
    c0x = stompy.kinematics.leg.x_with_vertical_calf(z)
    #h, t, k = stompy.kinematics.leg.point_to_angles(
    #    c0x, 0, z)
    #print(numpy.degrees(stompy.kinematics.leg.angles_to_calf_angle(h, t, k)))

    #tc = target_circle(tx, ty, c0x, 0.)
    tc = {
        'center': (tx, ty),
        'radius': numpy.sqrt((tx - c0x) ** 2. + (ty - 0.) ** 2.),
    }
    #lcp, rcp, hp0, hp1 = find_limits(tc, z, leg_number)
    ipts = stompy.kinematics.leg.find_limit_intersections(tc, z, leg_number)
    sp = swing_position_from_intersections(
        [tx, ty], rspeed, [c0x, 0], ipts, step_ratio)

    t1 = time.time()
    print("Time to compute swing point: %s" % (t1 - t0))
    pylab.plot(lpts[:, 0], lpts[:, 1])
    pylab.scatter(x, y, color='b')
    pylab.scatter(c0x, 0., color='r')
    pylab.gca().set_aspect(1.0)
    c = matplotlib.patches.Circle(
        tc['center'], tc['radius'], facecolor='none', edgecolor='g')
    pylab.gca().add_artist(c)

    if len(ipts):
        ipts = numpy.array(ipts)
        pylab.scatter(ipts[:, 0], ipts[:, 1], color='m')
    pylab.scatter(sp[0], sp[1], color='k', s=100)
    pylab.show()

if __name__ == '__main__':
    sp = calculate_swing_target(
        tx, ty, z, leg_number, rspeed, step_ratio,
        min_hip_distance=min_hip_distance,
        plot=True)
