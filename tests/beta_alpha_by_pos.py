#!/usr/bin/env python

import sys

import numpy
import pylab

import stompy

calf_angle_threshold = 14.0
if len(sys.argv) > 1:
    calf_angle_threshold = float(sys.argv[1])


kin = stompy.kinematics.leg.LegGeometry(1)


def xyz_to_calf_angle(x, y, z):
    #h, t, k = stompy.kinematics.leg.point_to_angles(x, y, z)
    h, t, k = kin.point_to_angles(x, y, z)
    return numpy.degrees(kin.angles_to_calf_angle(h, t, k))


def xyz_to_beta_alpha(x, y, z):
    h, t, k = kin.point_to_angles(x, y, z)
    # get angle of calf (to make beta)
    beta = numpy.pi / 2. - abs(kin.angles_to_calf_angle(h, t, k))
    # get alpha
    cl = kin.knee.to_cylinder_length(k)
    C = kin.knee.zero_angle
    B = numpy.arcsin(kin.knee.triangle_b / (cl / numpy.sin(C)))
    alpha = numpy.pi / 2. - abs(B)
    return numpy.cos(beta) / numpy.cos(alpha)


x_steps = 30.
zs = numpy.arange(0, -72, -1.)

pts = []

for z in zs:
    xmin, xmax = kin.limits_at_z_2d(z)
    xs = numpy.linspace(xmin, xmax, x_steps)
    for x in xs:
        ba = (xyz_to_beta_alpha(x, 0., z) * 760) / 3.14
        ca = xyz_to_calf_angle(x, 0., z)
        if (ca < 0):
            continue
        if (x < 10):
            continue
        pts.append([x, z, ba])

pts = numpy.array(pts)
print(pts[:, 2].min(), pts[:, 2].max())
cs = ['br'[int(abs(ca) > calf_angle_threshold)] for ca in pts[:, 2]]
sizes = pts[:, 2]
pylab.scatter(pts[:, 0], pts[:, 1], s=sizes, color=cs)
pylab.xlabel('foot x position')
pylab.ylabel('foot z position')
pylab.axhline(-35)
pylab.axhline(-40)
#pylab.title(
#    'blue = valid foot touch down positions with +-%s calf angle'
#    % calf_angle_threshold)
pylab.show()
