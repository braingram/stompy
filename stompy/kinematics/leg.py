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
import inspect

import numpy


default_cfg = {
    'hip': {
        'length': 11.0,
        'rest_angle': 0.,
        'cylinder_min': 16.0,  # TODO measure this!
        'cylinder_max': 23.153837,  # TODO measure this!
        'zero_angle_length': 19.8173,  # TODO measure this!
        'triangle_a': 6.83905,
        'triangle_b': 19.16327,
    },
    'thigh': {
        'length': 54.0,
        'rest_angle': numpy.radians(84.0),  # rest angle
        'cylinder_min': 24.0,
        'cylinder_max': 38.0,
        'triangle_a': 10.21631,
        'triangle_b': 33.43093,
    },
    'knee': {
        'length': 72.0,
        'rest_angle': numpy.radians(-83.0),  # rest angle
        'cylinder_min': 20.0,
        'cylinder_max': 32.0,
        'triangle_a': 7.4386,
        'triangle_b': 25.6021,
    },
}

# if not in here, use defaults
per_leg_cfg = {
    2: {
        'hip': {
            'cylinder_max': 24.0,  # TODO measure this!
            'zero_angle_length': 20.30,  # TODO measure this!
            'triangle_b': 19.62051,  # TODO measure this!
        },
    },
    5: {
        'hip': {
            'cylinder_max': 24.0,  # TODO measure this!
            'zero_angle_length': 20.30,  # TODO measure this!
            'triangle_b': 19.62051,  # TODO measure this!
        },
    },
}

# -- defines --
# length
# cylinder_min, cylinder_max
# triangle_a, triangle_b
# zero_angle
# rest_angle
# min_angle, max_angle
#
# base_beta (in firmware?)
# precompute (in firmware): thigh_length ** 2, knee_length ** 2, knee * thigh * -2


def merge_dicts(bd, md):
    for k in md:
        if k in bd and isinstance(md[k], dict):
            bd[k] = merge_dicts(bd[k], md[k])
        else:
            bd[k] = copy.deepcopy(md[k])
    return bd


class JointGeometry(object):
    def __init__(
            self, joint_type, length, rest_angle, cylinder_min, cylinder_max,
            triangle_a, triangle_b, zero_angle_length=None):
        self.joint_type = joint_type
        self.length = length
        self.rest_angle = rest_angle
        self.cylinder_min = cylinder_min
        self.cylinder_max = cylinder_max
        self.triangle_a = triangle_a
        self.triangle_b = triangle_b
        if zero_angle_length is None:
            if joint_type == 'hip':
                raise ValueError("Hip cylinder zero angle length must be measured")
            elif joint_type == 'thigh':
                zero_angle_length = cylinder_min
            elif joint_type == 'knee':
                zero_angle_length = cylinder_max
        self.zero_angle_length = zero_angle_length

        # compute zero angle
        self.zero_angle = self.raw_angle(self.zero_angle_length)

        # compute limits
        self.min_angle = self.joint_angle(self.cylinder_min)
        self.max_angle = self.joint_angle(self.cylinder_max)
        # if this is the hip, there is an opposing cylinder
        # so take shortest excursion
        if self.joint_type == 'hip':
            self.max_angle = min(abs(self.min_angle), abs(self.max_angle))
            self.min_angle = -self.max_angle

    def raw_angle(self, cylinder_length):
        a = self.triangle_a
        b = self.triangle_b
        c = cylinder_length
        # cosine rule, given 3 sides (where c is cylinder)
        # compute angle C (joint angle)
        return numpy.arccos((a * a + b * b - c * c) / (2 * a * b))
    
    def joint_angle(self, cylinder_length):
        return self.raw_angle(cylinder_length) - self.zero_angle


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


def circle_line_segment_intersection(c, l0, l1):
    # l0 = (x, y)
    # c['center'](x, y), c['radius']
    cxy = numpy.array(c['center'])  # q
    cr = c['radius']  # r
    l0 = numpy.array(l0)  # p1
    l1 = numpy.array(l1)  # p2
    v = l1 - l0  # v
    a = numpy.dot(v, v)
    b = 2. * numpy.dot(v, (l0 - cxy))
    c = (
        numpy.dot(l0, l0) + numpy.dot(cxy, cxy) -
        2. * numpy.dot(l0, cxy) - (cr * cr))

    #t = (-b +- sqrt(b ** 2 - 4ac)) / 2a
    disc = (b * b) - 4 * a * c
    if disc < 0:
        return None, None
    disc = numpy.sqrt(disc)
    t0 = (-b + disc) / (2. * a)
    t1 = (-b - disc) / (2. * a)
    if 0 <= t0 <= 1:
        t0 = l0 + t0 * (l1 - l0)
    else:
        t0 = None
    if 0 <= t1 <= 1:
        t1 = l0 + t1 * (l1 - l0)
    else:
        t1 = None
    return t0, t1


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


class LegGeometry(object):
    def __init__(self, leg_number):
        cfg = copy.deepcopy(default_cfg)
        # combine with any per-leg measurements
        if leg_number in per_leg_cfg:
            cfg = merge_dicts(cfg, per_leg_cfg[leg_number])
        for jn in ('hip', 'thigh', 'knee'):
            jc = cfg[jn]
            jc['joint_type'] = jn
            # unpack settings based on argument spec
            args, _, _, _ = inspect.getargspec(JointGeometry.__init__)
            setattr(self, jn, JointGeometry(*[jc[n] for n in args[1:] if n in jc]))
        # base beta = pi - thigh_rest - knee_rest, see inverse kinematics
        self.base_beta = (numpy.pi - self.thigh.rest_angle + self.knee.rest_angle)
        # precompute
        # - limit circles 2d: thigh_min, thigh_max, knee_min, knee_max
        self.compute_limit_circles_2d()

    def compute_limit_circles_2d(self):
        self.limit_circles_2d = {}
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
            self.angles_to_points(0, self.thigh.min_angle, self.knee.min_angle)))
        kmax_radius = numpy.linalg.norm(lpts[2] - lpts[0])
        cx, _, cy = lpts[1]
        self.limit_circles_2d['thigh_min'] = {
            'center': (cx, cy), 'radius': self.knee.length}
        lpts = numpy.array(list(
            self.angles_to_points(0, self.thigh.max_angle, self.knee.max_angle)))
        kmin_radius = numpy.linalg.norm(lpts[2] - lpts[0])
        cx, _, cy = lpts[1]
        self.limit_circles_2d['thigh_max'] = {
            'center': (cx, cy), 'radius': self.knee.length}
        cx, cy = self.hip.length, 0.
        self.limit_circles_2d['knee_min'] = {
            'center': (cx, cy), 'radius': kmin_radius}
        self.limit_circles_2d['knee_max'] = {
            'center': (cx, cy), 'radius': kmax_radius}
        self.z_min = (
            self.limit_circles_2d['thigh_max']['center'][1] -
            self.limit_circles_2d['thigh_max']['radius'])
        self.z_max = max(
            circle_intersection(
                self.limit_circles_2d['knee_max'], self.limit_circles_2d['thigh_min']),
            key=lambda i: i[0])[1]

    def get_limits(self):
        return {
            'hip': (self.hip.min_angle, self.hip.max_angle),
            'thigh': (self.thigh.min_angle, self.thigh.max_angle),
            'knee': (self.knee.min_angle, self.knee.max_angle),
        }

    def angles_to_points(self, hip, thigh, knee):
        x = self.hip.length
        z = 0
        ch = numpy.cos(hip)
        sh = numpy.sin(hip)
        yield x * ch, x * sh, z

        a = self.thigh.rest_angle - thigh
        x += self.thigh.length * numpy.cos(a)
        z += self.thigh.length * numpy.sin(a)
        yield x * ch, x * sh, z

        a = self.knee.rest_angle - knee - thigh
        x += self.knee.length * numpy.cos(a)
        z += self.knee.length * numpy.sin(a)

        yield x * ch, x * sh, z

    def point_to_angles(self, x, y, z):
        # TODO doesn't work for x < 0
        l = (x * x + y * y) ** 0.5
        hip = numpy.arctan2(y, x)
        L = (z * z + (l - self.hip.length) * (l - self.hip.length)) ** 0.5
        a1 = numpy.arccos(-z / L)
        a2 = numpy.arccos((
            self.knee.length * self.knee.length
            - self.thigh.length * self.thigh.length - L * L) /
            (-2 * self.thigh.length * L))
        alpha = (a1 + a2)
        beta = numpy.arccos((
            L * L - self.knee.length * self.knee.length -
            self.thigh.length * self.thigh.length) /
            (-2 * self.knee.length * self.thigh.length))
        thigh = self.thigh.rest_angle - (alpha - numpy.pi / 2.)
        knee = self.base_beta - beta
        return hip, thigh, knee

    def angles_to_calf_angle(self, hip, thigh, knee):
        _, p1, p2 = self.angles_to_points(hip, thigh, knee)
        dx = p2[0] - p1[0]
        # invert dz to fix quadrant
        dz = -(p2[2] - p1[2])
        return numpy.arctan2(dx, dz)

    def xy_center_at_z(self, z):
        l, r = self.limits_at_z_2d(z)
        return ((l + r) / 2., 0.)


    def x_with_calf_angle(self, z, a):
        return (
            numpy.sqrt(1 - (
                (z + numpy.cos(float(a)) * self.knee.length)
                / self.thigh.length) ** 2)
            * self.thigh.length + self.hip.length
            + numpy.sin(a) * self.knee.length)


    def x_with_vertical_calf(self, z):
        return (
            numpy.sqrt(1 - (
                (float(z) + self.knee.length)
                / self.thigh.length) ** 2)
            * self.thigh.length + self.hip.length)

    def limits_at_z_2d(self, z):
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
        if z > self.z_max or z < self.z_min:
            return None, None
        # right point
        if z > 0:
            # use right of knee_max
            r = max(circle_point_at_y(self.limit_circles_2d['knee_max'], z))
        else:
            # use min of rights of knee_max and thigh_max
            r = min([
                max(circle_point_at_y(self.limit_circles_2d['knee_max'], z)),
                max(circle_point_at_y(self.limit_circles_2d['thigh_max'], z))])
        # left point
        tminxs = circle_point_at_y(self.limit_circles_2d['thigh_min'], z)
        kminxs = circle_point_at_y(self.limit_circles_2d['knee_min'], z)
        if tminxs is None and kminxs is None:  # below both
            l = min(circle_point_at_y(self.limit_circles_2d['thigh_max'], z))
        elif tminxs is None:
            l = max(kminxs)
        elif kminxs is None:
            l = max(tminxs)
        else:
            l = max([max(kminxs), max(tminxs)])
        return l, r


    def limits_at_z_3d(self, z, n_slices=11, wrap=True):
        l, r = self.limits_at_z_2d(z)
        if l is None or r is None:
            return None
        angles = numpy.linspace(self.hip.min_angle, self.hip.max_angle, n_slices)
        pts = []
        for a in angles:
            ch = numpy.cos(a)
            sh = numpy.sin(a)
            pts.append((l * ch, l * sh, z))
            pts.append((r * ch, r * sh, z))
        if wrap:
            return pts[::2] + pts[1::2][::-1] + [pts[0], ]
        return pts


    def limit_intersections(
            self, c, z, min_hip_distance=None, max_calf_angle=None):
        # get 'left' [closest to hip] and 'right' circles
        # get hip limits (sets +-y angle)
        # TODO account for z translation
        l, r = self.limits_at_z_2d(z)
        # if l is too close to hip, extend it out
        if min_hip_distance is not None:
            l = max(l, min_hip_distance)
        if max_calf_angle is not None:
            rcalf = self.x_with_calf_angle(z, max_calf_angle)
            if rcalf < r:
                r = rcalf
        lc = {'center': (0, 0), 'radius': l}
        rc = {'center': (0, 0), 'radius': r}
        # intersection with hip lines
        hmin = self.hip.min_angle
        hmax = self.hip.max_angle
        camin = numpy.cos(hmin)
        samin = numpy.sin(hmin)
        camax = numpy.cos(hmax)
        samax = numpy.sin(hmax)
        min_lix, min_liy = camin * l, samin * l
        min_rix, min_riy = camin * r, samin * r
        max_lix, max_liy = camax * l, samax * l
        max_rix, max_riy = camax * r, samax * r
        min_ci = circle_intersection(c, lc)
        max_ci = circle_intersection(c, rc)
        min_li0, min_li1 = circle_line_segment_intersection(
            c, [min_lix, min_liy], [min_rix, min_riy])
        max_li0, max_li1 = circle_line_segment_intersection(
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
