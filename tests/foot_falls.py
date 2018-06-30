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
    ipts = stompy.kinematics.leg.limit_intersections(
        tc, z, leg_number, min_hip_distance=min_hip_distance)
    #sp = swing_position_from_intersections(
    #    [tx, ty], rspeed, [c0x, 0], ipts, step_ratio)
    sp = stompy.leg.restriction.swing_position_from_intersections(
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
    sp = stompy.leg.restriction.swing_position_from_intersections(
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
