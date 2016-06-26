#!/usr/bin/env python

import numpy
import pylab

from stompy.kinematics.leg import *

# 0.279, 0, 0  # hip to thigh
# 0.143399, 0, 1.364  # thigh to knee
# 0.0016, 0, -0.8185  # knee to upper linkage
# 0.203, 0, 0  # upper linkage to calf lower
# -0.0171, 0, -0.6656  # calf lower to ankle

# x, z
#htt = [0.2794, 0]  # hip to thigh
#ttk = [0.144, 1.36402]  # thigh to knee
#ktl = [0.0016, -0.8185]  # knee to upper linkage
#ltc = [0.203, 0]  # upper linkage to calf lower
#ctc = [0.2418, -1.14935]  # simulated calf links with prismatic joint
#cta = [-0.0171, -0.6656]  # calf lower to ankle

plot = True

#links = numpy.array([htt, ttk, ktl, ltc, cta])
links = numpy.array([htt, ttk, ctc, cta])

#calf_links = numpy.array([ktl, ltc, cta]).sum(0)
calf_links = numpy.array([ctc, cta]).sum(0)
simplified_links = numpy.array([htt, ttk, calf_links])


if plot:
    x, y = 0, 0
    for l in links:
        nx, ny = (x + l[0], y + l[1])
        pylab.plot([x, nx], [y, ny], color='b')
        x = nx
        y = ny

if plot:
    x, y = 0, 0
    for l in simplified_links:
        nx, ny = (x + l[0], y + l[1])
        pylab.plot([x, nx], [y, ny], color='r')
        x = nx
        y = ny

    ax = pylab.gca()
    ax.set_aspect('equal')

# link lengths
link_lengths = numpy.linalg.norm(simplified_links, axis=1)
# link resting angles
link_angles = numpy.arctan2(simplified_links[:, 1], simplified_links[:, 0])
link_names = ('hip', 'thigh', 'knee')

for (n, l, a) in zip(link_names, link_lengths, link_angles):
    print("%s, %s[%s], %s[%s]" % (n, l, l * 39.3701, a, numpy.degrees(a)))

if plot:
    x, y = 0, 0
    for (l, a) in zip(link_lengths, link_angles):
        dy = l * numpy.sin(a)
        dx = l * numpy.cos(a)
        nx, ny = (x + dx, y + dy)
        pylab.plot([x, nx], [y, ny], color='g')
        x = nx
        y = ny

# the knee angle needs to also take into account the calf angle offset
angle_knee = numpy.radians(20)  # + tilts up

if plot:
    x, y, oa = 0, 0, 0
    for (n, l, a) in zip(link_names, link_lengths, link_angles):
        if n == 'knee':
            oa += angle_knee
        dy = l * numpy.sin(a + oa)
        dx = l * numpy.cos(a + oa)
        nx, ny = (x + dx, y + dy)
        pylab.plot([x, nx], [y, ny], color='m')
        x = nx
        y = ny

angle_thigh = numpy.radians(-20)  # - tilts up

if plot:
    x, y, oa = 0, 0, 0
    for (n, l, a) in zip(link_names, link_lengths, link_angles):
        if n == 'thigh':
            oa += angle_thigh
        elif n == 'knee':
            oa += angle_knee
        dy = l * numpy.sin(a + oa)
        dx = l * numpy.cos(a + oa)
        nx, ny = (x + dx, y + dy)
        pylab.plot([x, nx], [y, ny], color='k')
        x = nx
        y = ny

    pylab.show()
