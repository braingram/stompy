#!/usr/bin/env python
"""
Lower (while tracing xy)
    wait until done [or by load]
    compute stance (trace xy at current z)
Stance (trace xy at z)
    wait until done
    compute raise
Raise (while tracing xy)
    wait until done
    compute swing
Swing (trace xy at z)
    wait until done
    compute lower


XY can change at any time, when changed the current action
will need to be recomputed and a modified swing will be needed
to recenter the foot path (so fx, fy is optimal)

Fancy things like load balancing and terrain adaptation are
ignored for now.
"""

import numpy

from . import computer


def point_on_circle(cx, cy, radius, angle):
    return (
        cx + radius * numpy.cos(angle),
        cy + radius * numpy.sin(angle),
    )


def generate_paths(
        cx, cy, fx, fy, length,
        hz=0.8, lz=1.2,
        t=10., dt=0.1, phase=0.,
        st=0.85, pt=0.1):

    if length < 0:
        reverse = True
        length = -length
    else:
        reverse = False
    # do everything with a 0-1 cycle (set by phase)
    # place, stance, lift, swing

    # place: 0 -> (st * pt)
    place_p = (0, st * pt)
    to_place_p = lambda p: p / place_p[1]
    # stance: (st * pt) -> (st * (1 - pt))
    stance_p = (place_p[1], (st * (1. - pt)))
    to_stance_p = lambda p: (p - stance_p[0]) / (stance_p[1] - stance_p[0])
    # lift: (st * (1 - pt)) -> st
    lift_p = (stance_p[1], st)
    to_lift_p = lambda p: (p - lift_p[0]) / (lift_p[1] - lift_p[0])
    # swing: st -> 1
    swing_p = (lift_p[1], 1)
    to_swing_p = lambda p: (p - swing_p[0]) / (swing_p[1] - swing_p[0])
    # handle reverse by setting xy
    if reverse:
        to_xyp = lambda p: 1. - p / st
    else:
        to_xyp = lambda p: p / st

    # TODO identify current state by phase, setup to match that point
    if phase < stance_p[0]:  # in place
        xyfr = to_xyp(phase)
    elif phase < lift_p[0]:  # in stance
        xyfr = to_xyp(phase)
    elif phase < swing_p[0]:  # in lift
        xyfr = to_xyp(phase)
    else:  # in swing
        xyfr = 1 - to_swing_p(phase)

    # setup xy path
    xyp = computer.Path(fx, fy)
    xyp.arc(cx, cy, length, fr=xyfr)
    # setup z easing
    zp = computer.Path(hz, 0, easing=False)
    zp.line(0, lz - hz, fr=0.)  # just use column 0 for z

    ts = numpy.arange(0, t, dt)
    n = ts.size
    phases = numpy.linspace(0, 1., ts.size)

    # TODO skip ahead to current phase
    pts = []
    for i in xrange(n):
        t = ts[i]
        p = phases[i]
        if p < stance_p[0]:  # place
            # place: z down (smoothed), xy start following trajectory
            lp = to_place_p(p)
            xy = xyp(to_xyp(p))
            z = zp(lp)[0]
            state = 0
        elif p < lift_p[0]:  # stance
            # stance: no z, continue following xy
            lp = to_stance_p(p)
            xy = xyp(to_xyp(p))
            z = zp(1.)[0]
            state = 1
        elif p < swing_p[0]:  # lift
            # lift: z up (smoothed), end following xy
            lp = to_lift_p(p)
            xy = xyp(to_xyp(p))
            z = zp(1. - lp)[0]
            state = 2
        else:  # swing
            # swing: no z, follow complete xy backwards at max speed
            lp = to_swing_p(p)
            xy = xyp(1-lp)
            z = zp(0.)[0]
            state = 3
        #pts.append((xy[0], xy[1], z, p, state, t))
        pts.append((xy[0], xy[1], z))

    # adjust for current phase

    # set current phase
    return set_cycle_phase(pts, phase)


def generate(
        cx, cy, fx, fy, phi,
        hz=0.8, lz=1.2,
        t=10., dt=0.1, phase=0,
        st=0.85, pt=0.1):
    """
    hz: 'high' z foot position
    lz: 'low' z foot position
    cx, cy: circle center location
    fx, fy: foot center location
    phi: arc length of foot cycle (meters)
    t: cycle time
    dt: cycle resolution
    phase: cycle phase [0,1]
    """
    if phi < 0.:  # reverse
        reverse = True
        phi = -phi
    else:
        reverse = False
    radius = ((fx - cx) ** 2. + (fy - cy) ** 2.) ** 0.5
    # TODO if radius > some threshold, just go straight
    # TODO if radius == 0, don't move foot
    if radius == 0:
        radius = 0.00001
    #print("radius: %s" % radius)
    foot_angle = numpy.arctan2((fy - cy), (fx - cx))
    #print("foot_angle: %s" % numpy.degrees(foot_angle))
    # convert phi (in meters) to radians
    # phi / r = phi_angles
    phi_a = phi / radius

    ts = numpy.arange(0, t, dt)
    stance_t = t * st
    place_t = stance_t * pt
    lift_t = stance_t * (1. - pt)
    #print("stance_t: %s" % stance_t)
    #print("place_t: %s" % place_t)
    #print("lift_t: %s" % lift_t)

    n_stance = stance_t / dt + 1
    n_swing = (t - stance_t) / dt
    da_stance = -phi_a / n_stance
    da_swing = phi_a / n_swing
    #print("n_stance: %s" % n_stance)
    #print("n_swing: %s" % n_swing)
    #print("da_stance: %s" % da_stance)
    #print("da_swing: %s" % da_swing)

    pts = []
    angle = foot_angle + phi_a / 2.  # starting position
    x, y = point_on_circle(cx, cy, radius, angle)
    z = hz
    zr = hz - lz
    dz = zr / (place_t / dt)
    # dx/dy_stance
    # dx/dy_swing
    state = 0
    #pts.append((x, y, z, state, p))
    pts.append((x, y, z))
    for (i, t) in enumerate(ts):
        if state == 0:  # stance
            angle += da_stance
            if t < place_t:
                z -= dz
            elif t > lift_t:
                z += dz
            if t >= stance_t:
                state = 1  # swing
        else:  # swing
            angle += da_swing
            z = hz
        x, y = point_on_circle(cx, cy, radius, angle)
        #pts.append((x, y, z, state, p))
        pts.append((x, y, z))
    pts = set_cycle_phase(pts, phase)
    if reverse:
        return pts[::-1]
    return pts


def generate_cycle(
        hz=0.8, lz=1.2, fx=1., rx=-1.,
        #hz=-0.5, lz=-0.8, fx=1., rx=-1.,
        y=2.5, t=10., dt=0.1, ratio=0):
    """
    all coordinates in body frame
    hz = 'high' z
    lz = 'low' z
    fx = 'forward' foot x position
    rx = 'rear' foot x position
    y = foot y position
    t = cycle time
    dt = cycle time resolution
    ration = cycle phase [0,1]
    """
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
