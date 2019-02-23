#!/usr/bin/env python

import numpy
import pylab

import stompy


leg_number = 0
limits_eps = 3.0
calf_eps = 3.0
res_threshold = 0.6
res_max_threshold = 0.9
calf_max_angle = numpy.radians(30)

limits = stompy.geometry.get_limits(0)


def xyz_to_res(x, y, z):
    angles = stompy.kinematics.leg.point_to_angles(x, y, z)
    return stompy.restriction.leg.calculate_restriction(
        (x, y, z), {
            'hip': angles[0],
            'thigh': angles[1],
            'knee': angles[2],
        },
        limits, limits_eps,
        calf_eps, calf_max_angle)


x_steps = 30.
zs = numpy.arange(0, -72, -1.)

pts = []

for z in zs:
    xmin, xmax = stompy.kinematics.leg.limits_at_z_2d(z)
    xmin = max(0., xmin)
    xs = numpy.linspace(xmin, xmax, x_steps)
    for x in xs:
        v = xyz_to_res(x, 0., z)
        pts.append([x, z, v])

pts = numpy.array(pts)
cs = []
ss = []
for pt in pts:
    if pt[2] > res_max_threshold:
        cs.append('r')
    elif pt[2] > res_threshold:
        cs.append('b')
    else:
        cs.append('g')
    ss.append(pt[2] * 50 + 1)
# TODO size
pylab.scatter(pts[:, 0], pts[:, 1], color=cs, s=ss)
pylab.xlabel('foot x position')
pylab.ylabel('foot z position')
pylab.title(
    'blue = res > %s' % res_threshold)
pylab.show()
