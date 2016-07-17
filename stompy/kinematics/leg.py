#!/usr/bin/env python
"""

Angles [gazebo]
    hip angle
        looking down on robot + is CCW
        0 resting
    thigh angle
        looking at side with leg pointed right, + tilts down
        0 resting
        angles always +
    calf angle
        looking at side with leg pointed right, - tilts up
        0 resting
        angles always -
Positions [gazebo] in leg space
    x
        looking at side with leg pointed right + is out (right)
        never -
        minimum is... (depends on z)
    y
        looking down on leg pointed right + is up, - is down
        positive and negative
    z
        looking at side with leg pointed right, + is down
        0 is in line with hip joint, below this is +
"""

import numpy

# meters
htt = [0.279, 0]  # hip to thigh
ttk = [0.143, 1.364]  # thigh to knee
ktl = [0.0016, -0.8185]  # knee to upper linkage
ltc = [0.203, 0]  # upper linkage to calf lower
ktc = [0.203, -0.847]  # simulated calf links with prismatic joint
cta = [0.02, -0.968]  # calf lower to ankle
kt = [ktc[0] + cta[0], ktc[1] + cta[1]]

hip_limits = (-0.70616022, 0.70616022)
hip_link = 0.279  # coxa

#thigh_link = 1.372  # femer
thigh_link = numpy.linalg.norm(ttk)  # femer
thigh_limits = (0., 1.5708)
#thigh_rest_angle = numpy.radians(84)
thigh_rest_angle = numpy.arctan2(ttk[1], ttk[0])

#calf_link = 1.829
knee_link = numpy.linalg.norm(kt)
knee_limits = (-2.3736, 0.)
#calf_rest_angle = -numpy.radians(83)
knee_rest_angle = numpy.arctan2(kt[1], kt[0])


def in_limits(angle, limits):
    if angle < limits[0]:
        return False
    if angle > limits[1]:
        return False
    return True


def inverse(x, y, z):
    l = numpy.sqrt(x * x + y * y)

    # TODO deal with -x
    hip_angle = numpy.arctan2(y, x)

    L = numpy.sqrt(z ** 2. + (l - hip_link) ** 2.)
    a1 = numpy.arccos(z / L)
    a2 = numpy.arccos(
        (knee_link ** 2. - thigh_link ** 2. - L ** 2.) /
        (-2 * thigh_link * L))
    alpha = (a1 + a2)

    beta = numpy.arccos(
        (L ** 2 - knee_link ** 2 - thigh_link ** 2) /
        (-2 * knee_link * thigh_link))
    #print 'point:', x, y, z
    #print 'a1', a1, numpy.degrees(a1)
    #print 'a2', a2, numpy.degrees(a2)
    #print 'alpha', alpha, numpy.degrees(alpha)
    #print 'beta', beta, numpy.degrees(beta)
    #print 'L', L

    thigh_angle = alpha - numpy.pi / 2.
    thigh_angle = thigh_rest_angle - thigh_angle
    base_beta = numpy.pi - thigh_rest_angle + knee_rest_angle
    knee_angle = base_beta - beta

    return hip_angle, thigh_angle, knee_angle


def forward(hip_angle, thigh_angle, knee_angle):
    x = hip_link
    y = 0
    z = 0
    #print "angles:", hip_angle, thigh_angle, knee_angle
    #x += hip_link * numpy.cos(hip_angle)
    #y += hip_link * numpy.sin(hip_angle)

    # thigh + is down in gazebo
    a = thigh_rest_angle - thigh_angle
    x += thigh_link * numpy.cos(a)
    z += thigh_link * numpy.sin(a)

    a = knee_rest_angle - knee_angle - thigh_angle
    x += knee_link * numpy.cos(a)
    z += knee_link * numpy.sin(a)

    # rotate about hip angle
    y = x * numpy.sin(hip_angle)
    x *= numpy.cos(hip_angle)
    return x, y, -z


def compute_range_of_movements():
    hl = hip_limits[0] + 0.01, hip_limits[1] - 0.01, 50
    tl = thigh_limits[0] + 0.01, thigh_limits[1] - 0.01, 10
    kl = knee_limits[0] + 0.01, knee_limits[1] - 0.01, 10
    pts = []
    for h in numpy.linspace(*hl):
        for t in numpy.linspace(*tl):
            for k in numpy.linspace(*kl):
                x, y, z = forward(h, t, k)
                if x < hip_link:
                    continue
                ch, ct, ck = inverse(x, y, z)
                if max((abs(h - ch), abs(t - ct), abs(k - ck))) > 0.001:
                    raise Exception
                pts.append([h, t, k, x, y, z])
    return pts


def compute_error():
    hl = hip_limits[0] + 0.01, hip_limits[1] - 0.01, 10
    tl = thigh_limits[0] + 0.01, thigh_limits[1] - 0.01, 10
    kl = knee_limits[0] + 0.01, knee_limits[1] - 0.01, 10
    err = []
    for h in numpy.linspace(*hl):
        for t in numpy.linspace(*tl):
            for k in numpy.linspace(*kl):
                x, y, z = forward(h, t, k)
                if x < hip_link:
                    continue
                ch, ct, ck = inverse(x, y, z)
                if numpy.isfinite(ch) and numpy.isfinite(h):
                    e = abs(ch - h)
                    ej = 0
                else:
                    e = 100
                    ej = 0
                if numpy.isfinite(ct) and numpy.isfinite(t):
                    if abs(ct - t) > e:
                        e = abs(ct - t)
                        ej = 1
                else:
                    e = 100
                    ej = 1
                if numpy.isfinite(ck) and numpy.isfinite(k):
                    if abs(ck - k) > e:
                        e = abs(ck - k)
                        ej = 2
                else:
                    e = 100
                    ej = 2
                err.append([h, t, k, x, y, z, ch, ct, ck, e, ej])
    err = numpy.array(err)
    return err, err.mean(), err[:, 9].max()
    #return numpy.abs(err)
