#!/usr/bin/env python

import numpy

from .. import consts
from .. import geometry


def angles_to_calf_angle(hip, thigh, knee):
    _, p1, p2 = angles_to_points(hip, thigh, knee)
    dx = p2[0] - p1[0]
    # invert dz to fix quadrant
    dz = -(p2[2] - p1[2])
    return numpy.arctan2(dx, dz)


def angles_to_points(hip, thigh, knee):
    x = geometry.HIP_LENGTH
    z = 0
    ch = numpy.cos(hip)
    sh = numpy.sin(hip)
    yield x * ch, x * sh, z

    a = geometry.THIGH_REST_ANGLE - thigh
    x += geometry.THIGH_LENGTH * numpy.cos(a)
    z += geometry.THIGH_LENGTH * numpy.sin(a)
    yield x * ch, x * sh, z

    a = geometry.KNEE_REST_ANGLE - knee - thigh
    x += geometry.KNEE_LENGTH * numpy.cos(a)
    z += geometry.KNEE_LENGTH * numpy.sin(a)

    yield x * ch, x * sh, z


def point_to_angles(x, y, z):
    l = (x * x + y * y) ** 0.5
    hip = numpy.arctan2(y, x)
    L = (z * z + (l - geometry.HIP_LENGTH) * (l - geometry.HIP_LENGTH)) ** 0.5
    a1 = numpy.arccos(-z / L)
    a2 = numpy.arccos((
        geometry.KNEE_LENGTH * geometry.KNEE_LENGTH
        - geometry.THIGH_LENGTH * geometry.THIGH_LENGTH - L * L) /
        (-2 * geometry.THIGH_LENGTH * L))
    alpha = (a1 + a2)
    beta = numpy.arccos((
        L * L - geometry.KNEE_LENGTH * geometry.KNEE_LENGTH -
        geometry.THIGH_LENGTH * geometry.THIGH_LENGTH) /
        (-2 * geometry.KNEE_LENGTH * geometry.THIGH_LENGTH))
    thigh = geometry.THIGH_REST_ANGLE - (alpha - numpy.pi / 2.)
    knee = geometry.BASE_BETA - beta
    return hip, thigh, knee


def circle_intersection(c0, c1):
    x0, y0 = c0['center']
    r0 = c0['radius']
    x1, y1 = c1['center']
    r1 = c1['radius']
    dx = (x1 - x0)
    dy = (y1 - y0)
    d = (dx * dx + dy * dy) ** 0.5
    if d > (r0 + r1):
        return None
    if d < abs(r0 - r1):
        return None
    if r0 == r1 and d == 0:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = (r0 * r0 - a * a) ** 0.5
    x2 = x0 + a * dx / d
    y2 = y0 + a * dy / d
    x3p = x2 + h * dy / d
    y3p = y2 - h * dx / d
    x3n = x2 - h * dy / d
    y3n = y2 + h * dx / d
    return ((x3p, y3p), (x3n, y3n))


def circle_point_at_y(c, y):
    # (x - cx) ** 2 + (y - cy) ** 2 = r ** 2
    cx, cy = c['center']
    r = c['radius']
    if abs(y - cy) > r:
        return None
    # (x - cx) ** 2 = r ** 2 - (y - cy) ** 2
    # +-(x - cx) = (r ** 2 - (y - cy) ** 2) ** 0.5
    # x = (r ** 2 - (y - cy) ** 2) ** 0.5 +- cx
    dx = (r ** 2 - (y - cy) ** 2) ** 0.5
    return dx + cx, cx - dx


limit_circles_2d = {}
# thigh_min: blue: thigh min, knee sweep
#  radius = knee_length
#  center = computed thigh min pt[1]
# thigh_max: green: thigh max, knee sweep
#  radius = knee_length
#  center = computed thigh max pt[1]
# knee_min: orange: knee min, thigh sweep
#  radius = computed hip pt[0] to angle pt[2]
#  center = hip_length in x
# knee_max: red: knee max, thigh sweep
#  radius = computed hip pt[0] to angle pt[2]
#  center = hip_length in x
lpts = numpy.array(list(
    angles_to_points(0, geometry.THIGH_MIN_ANGLE, geometry.KNEE_MIN_ANGLE)))
kmax_radius = numpy.linalg.norm(lpts[2] - lpts[0])
cx, _, cy = lpts[1]
limit_circles_2d['thigh_min'] = {
    'center': (cx, cy), 'radius': geometry.KNEE_LENGTH}
lpts = numpy.array(list(
    angles_to_points(0, geometry.THIGH_MAX_ANGLE, geometry.KNEE_MAX_ANGLE)))
kmin_radius = numpy.linalg.norm(lpts[2] - lpts[0])
cx, _, cy = lpts[1]
limit_circles_2d['thigh_max'] = {
    'center': (cx, cy), 'radius': geometry.KNEE_LENGTH}
cx, cy = geometry.HIP_LENGTH, 0.
limit_circles_2d['knee_min'] = {
    'center': (cx, cy), 'radius': kmin_radius}
limit_circles_2d['knee_max'] = {
    'center': (cx, cy), 'radius': kmax_radius}
z_min = (
    limit_circles_2d['thigh_max']['center'][1] -
    limit_circles_2d['thigh_max']['radius'])
z_max = max(
    circle_intersection(
        limit_circles_2d['knee_max'], limit_circles_2d['thigh_min']),
    key=lambda i: i[0])[1]
del cx, cy, lpts, kmin_radius, kmax_radius


def limits_at_z_2d(z):
    """
    to find right edge:
    - find point on knee_max at z (and x > 0)
    - find point on thigh_max at z (and x > 0)
    - if z > 0, if no thigh_max or knee_max > z use knee_max
      else, if thigh_max < knee_max, use thigh_max, else use knee_max

    for left [cuts off a small region under the hip]
    - find point on thigh_min at z (with largest x)
    - find point on knee_min at z (with largest x)
    - take largest x of thigh_min and knee_min
    - if outside thigh_min and knee_min
     find point on thigh_max at z (with smallest x)
    """
    if z > z_max or z < z_min:
        return None, None
    # right point
    if z > 0:
        # use right of knee_max
        r = max(circle_point_at_y(limit_circles_2d['knee_max'], z))
    else:
        # use min of rights of knee_max and thigh_max
        r = min([
            max(circle_point_at_y(limit_circles_2d['knee_max'], z)),
            max(circle_point_at_y(limit_circles_2d['thigh_max'], z))])
    # left point
    tminxs = circle_point_at_y(limit_circles_2d['thigh_min'], z)
    kminxs = circle_point_at_y(limit_circles_2d['knee_min'], z)
    if tminxs is None and kminxs is None:  # below both
        l = min(circle_point_at_y(limit_circles_2d['thigh_max'], z))
    elif tminxs is None:
        l = max(kminxs)
    elif kminxs is None:
        l = max(tminxs)
    else:
        l = max([max(kminxs), max(tminxs)])
    return l, r


def limits_at_z_3d(z, leg_number, n_slices=11, wrap=True):
    l, r = limits_at_z_2d(z)
    if l is None or r is None:
        return None
    if leg_number in consts.MIDDLE_LEGS:
        hmin, hmax = (
            geometry.HIP_MIDDLE_MIN_ANGLE, geometry.HIP_MIDDLE_MAX_ANGLE)
    else:
        hmin, hmax = geometry.HIP_MIN_ANGLE, geometry.HIP_MAX_ANGLE
    angles = numpy.linspace(hmin, hmax, n_slices)
    pts = []
    for a in angles:
        ch = numpy.cos(a)
        sh = numpy.sin(a)
        pts.append((l * ch, l * sh, z))
        pts.append((r * ch, r * sh, z))
    if wrap:
        return pts[::2] + pts[1::2][::-1] + [pts[0], ]
    return pts
