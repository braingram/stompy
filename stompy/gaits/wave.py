#!/usr/bin/env python

import numpy


def generate_cycle(
        hz=0.8, lz=1.2, fx=1., rx=-1.,
        #hz=-0.5, lz=-0.8, fx=1., rx=-1.,
        y=2.5, t=10., dt=0.1, ratio=0):
    ts = numpy.arange(0, t, dt)
    xr = fx - rx
    zr = hz - lz
    stance_t = t * 0.85
    place_t = stance_t * 0.1
    lift_t = stance_t * 0.9
    # 2/3 time on ground, from fx to rx
    # 1/3 time off ground, from rx to fx
    # start at front
    # - moving backward
    # - put foot down over 1/5 of stance
    # move backward
    # - lift when 4/5 of stance
    # move forward
    pts = []
    x = fx
    z = hz
    dz = zr / (place_t / dt)
    dx_stance = -xr / ((stance_t / dt) + 1)
    dx_swing = xr / ((t - stance_t) / dt)
    state = 0  # stance
    pts.append((x, y, z))
    for t in ts:
        if state == 0:  # stance
            x += dx_stance
            if t < place_t:
                z -= dz
            elif t > lift_t:
                z += dz
            if t >= stance_t:
                state = 1  # swing
        else:  # swing
            x += dx_swing
            z = hz
        pts.append((x, y, z))
    return set_cycle_phase(pts, ratio)


def set_cycle_phase(pts, ratio):
    ir = int(len(pts) * ratio)
    return numpy.roll(pts, ir, axis=0)
