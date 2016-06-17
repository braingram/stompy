#!/usr/bin/env python
"""
Forward
x = cos(theta1) * [L1 + L2*cos(theta2) + L3*cos(theta2 + theta3 -180deg)]
y = x * tan(theta1)
z = [L2 * sin(theta2)] + [L3 * sin(theta2 + theta3 - 180deg)]
"""

import numpy

# meters
htt = [0.279, 0]  # hip to thigh
ttk = [0.143399, 1.364]  # thigh to knee
ktl = [0.0016, -0.8185]  # knee to upper linkage
ltc = [0.203, 0]  # upper linkage to calf lower
cta = [-0.0171, -0.6656]  # calf lower to ankle

hip_limits = (-0.70616022, 0.70616022)
hip_link = 0.279  # coxa
thigh_link = 1.37158  # femer
thigh_limits = (0., 1.5708)
thigh_rest_angle = 1.46605
#numpy.pi / 2. - numpy.arctan2(1.364, 0.14399)
#thigh_rest = 0.10517498504872225
#calf_link = 1.828  # tibia, varies with load
calf_link = 1.49589
calf_limits = (-2.3736, 0.)
calf_rest_angle = -1.44512
# 0.279, 0, 0  # hip to thigh
# 0.143399, 0, 1.364  # thigh to knee

# 0.0016, 0, -0.8185  # knee to upper linkage
# 0.203, 0, 0  # upper linkage to calf lower
# -0.0171, 0, -0.6656  # calf lower to ankle
#knee_limits = (-2.3736, 0.)


def in_limits(angle, limits):
    if angle < limits[0]:
        return False
    if angle > limits[1]:
        return False
    return True


def inverse(x, y, z):
    l = numpy.sqrt(x * x + y * y)

    hip_angle = numpy.arctan2(y, x)

    L = numpy.sqrt(z ** 2. + (l - hip_link) ** 2.)
    a1 = numpy.arccos(z / L)
    a2 = numpy.arccos(
        (calf_link ** 2. - thigh_link ** 2. - L ** 2.) /
        (-2 * thigh_link * L))
    alpha = (a1 + a2)

    beta = numpy.arccos(
        (L ** 2 - calf_link ** 2 - thigh_link ** 2) /
        (-2 * calf_link * thigh_link))
    #print 'point:', x, y, z
    #print 'a1', a1, numpy.degrees(a1)
    #print 'a2', a2, numpy.degrees(a2)
    #print 'alpha', alpha, numpy.degrees(alpha)
    #print 'beta', beta, numpy.degrees(beta)
    #print 'L', L

    thigh_angle = alpha - numpy.pi / 2.
    thigh_angle = thigh_rest_angle - thigh_angle
    base_beta = numpy.pi - thigh_rest_angle + calf_rest_angle
    calf_angle = base_beta - beta

    return hip_angle, thigh_angle, calf_angle


def forward(hip_angle, thigh_angle, calf_angle):
    x = hip_link
    y = 0
    z = 0
    #print "angles:", hip_angle, thigh_angle, calf_angle
    #x += hip_link * numpy.cos(hip_angle)
    #y += hip_link * numpy.sin(hip_angle)

    # thigh + is down in gazebo
    a = thigh_rest_angle - thigh_angle
    x += thigh_link * numpy.cos(a)
    z += thigh_link * numpy.sin(a)

    a = calf_rest_angle - calf_angle - thigh_angle
    x += calf_link * numpy.cos(a)
    z += calf_link * numpy.sin(a)

    # rotate about hip angle
    y = x * numpy.sin(hip_angle)
    x *= numpy.cos(hip_angle)
    return x, y, -z


def compute_error():
    hl = hip_limits[0] + 0.01, hip_limits[1] - 0.01, 10
    tl = thigh_limits[0] + 0.01, thigh_limits[1] - 0.01, 10
    kl = calf_limits[0] + 0.01, calf_limits[1] - 0.01, 10
    err = []
    for h in numpy.linspace(*hl):
        for t in numpy.linspace(*tl):
            for k in numpy.linspace(*kl):
                x, y, z = forward(h, t, k)
                ch, ct, ck = inverse(x, y, z)
                if numpy.isfinite(ch) and numpy.isfinite(h):
                    err.append(ch - h)
                else:
                    err.append(100)
                if numpy.isfinite(ct) and numpy.isfinite(t):
                    err.append(ct - t)
                else:
                    err.append(100)
                if numpy.isfinite(ck) and numpy.isfinite(k):
                    err.append(ck - k)
                else:
                    err.append(100)
    err = numpy.array(err)
    aerr = numpy.abs(err)
    return aerr.mean(), aerr.max()
    #return numpy.abs(err)
