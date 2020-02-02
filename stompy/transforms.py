#!/usr/bin/env python

import math

import numpy


def translation_2d(x, y):
    return numpy.matrix(
        [[1., 0., x], [0., 1., y], [0., 0., 1.]])


def rotation_2d(a, degrees=False):
    if degrees:
        a = math.radians(a)
    sa = math.sin(a)
    ca = math.cos(a)
    return numpy.matrix(
        [[ca, -sa, 0.], [sa, ca, 0.], [0., 0., 1.]])


def affine_2d(x, y, a, degrees=False):
    #return rotation(a, degrees) * translation(x, y)
    return translation_2d(x, y) * rotation_2d(a, degrees)


def rotation_about_point_2d(x, y, a, degrees=False):
    return (
        translation_2d(x, y) *
        rotation_2d(a, degrees) *
        translation_2d(-x, -y))


def transform_2d(m, x, y):
    r = m * [[x], [y], [1.]]
    return float(r[0]), float(r[1])


def homogeneous_2d(pts):
    """Expects input of [npts, dims]"""
    pts = numpy.array(pts)
    return numpy.hstack((
        pts, numpy.ones((pts.shape[0], 1)))).T


def nonhomogeneous_2d(pts):
    """Expects input of [dims, npts]"""
    return numpy.array(pts[:2].T)


def transform_2d_array(m, pts):
    r = m * homogeneous_2d(pts)
    return nonhomogeneous_2d(r)


def translation_3d(x, y, z):
    return numpy.matrix([
        [1.0, 0., 0., x],
        [0., 1.0, 0., y],
        [0., 0., 1.0, z],
        [0., 0., 0., 1.]])


def rotation_3d(xa, ya, za, degrees=False):
    """Applied as x, y, z"""
    if xa != 0.:
        if degrees:
            xa = math.radians(xa)
        sx = math.sin(xa)
        cx = math.cos(xa)
        rx = numpy.matrix([
            [1.0, 0., 0., 0.],
            [0., cx, -sx, 0.],
            [0., sx, cx, 0.],
            [0., 0., 0., 1.]])
    else:
        rx = numpy.matrix(numpy.identity(4, dtype='f8'))
    if ya != 0.:
        if degrees:
            ya = math.radians(ya)
        sy = math.sin(ya)
        cy = math.cos(ya)
        ry = numpy.matrix([
            [cy, 0., sy, 0.],
            [0., 1., 0., 0.],
            [-sy, 0., cy, 0.],
            [0., 0., 0., 1.]])
    else:
        ry = numpy.matrix(numpy.identity(4, dtype='f8'))
    if za != 0.:
        if degrees:
            za = math.radians(za)
        sz = math.sin(za)
        cz = math.cos(za)
        rz = numpy.matrix([
            [cz, -sz, 0., 0.],
            [sz, cz, 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]])
    else:
        rz = numpy.matrix(numpy.identity(4, dtype='f8'))
    #return rx * ry * rz
    return rz * ry * rx


def affine_3d(x, y, z, xa, ya, za, degrees=False):
    """Applied as rotation then translation"""
    #return rotation(a, degrees) * translation(x, y)
    return (
        translation_3d(x, y, z) *
        rotation_3d(xa, ya, za, degrees))


def rotation_about_point_3d(x, y, z, xa, ya, za, degrees=False):
    return (
        translation_3d(x, y, z) *
        rotation_3d(xa, ya, za, degrees) *
        translation_3d(-x, -y, -z))


def transform_3d(m, x, y, z):
    r = m * [[x], [y], [z], [1.]]
    return float(r[0]), float(r[1]), float(r[2])


def homogeneous_3d(pts):
    """Expects input of [npts, dims]"""
    pts = numpy.array(pts)
    return numpy.hstack((
        pts, numpy.ones((pts.shape[0], 1)))).T


def nonhomogeneous_3d(pts):
    """Expects input of [dims, npts]"""
    return numpy.array(pts[:3].T)


def transform_3d_array(m, pts):
    r = m * homogeneous_3d(pts)
    return nonhomogeneous_3d(r)


def blend(t0, t1, n):
    """warning! infinite generator"""
    t = t0.copy()
    dt = (t1 - t0) / float(n)
    i = 0
    while i < n:
        t += dt
        yield t
        i += 1
    while True:
        yield t1
