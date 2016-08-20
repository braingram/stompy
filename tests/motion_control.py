#!/usr/bin/env python
"""
S-curves have 7 phases
- increase acceleration
- constant acceleration
- decrease acceleration
- constant velocity
- increase deceleration
- constant deceleration
- decrease deceleration

case 1
 - accel
    v = v_s + acc * t
    e = 0.5 * acc * t * t + v_s * t + s
case 2
 - accel cruise
    e = 0.5 * acc * a_t * a_t + v_s * a_t + s + v_c * c_t
case 3
 - accel then decel
    e = 0.5 * acc * a_t * a_t + v_s * a_t + s - 0.5 * dec * d_t * d_t - v_e * d_t
 - accel cruise then decel

don't enforce max accel

for polynomial paths
http://www.springer.com/cda/content/document/cda_downloaddocument/9783540856283-c1.pdf?SGWID=0-0-45-615213-p173839603
"""

import numpy
import pylab


approximate = lambda v, t, e=0.001: abs(v - t) < e


def plot(ts, vs, xs, case, e, t, v_e):
    pylab.subplot(211)
    pylab.plot(ts, xs, color='b')
    pylab.scatter(ts, xs, color='b')
    pylab.axhline(e, color='b')
    pylab.axvline(t, color='k')
    xl = pylab.xlim()
    pylab.xlim(xl[0], xl[1]+0.5)
    yl = pylab.ylim()
    pylab.ylim(yl[0], yl[1]+0.5)
    pylab.title("Case %s: positon vs time" % case)
    pylab.subplot(212)
    pylab.plot(ts, vs, color='g')
    pylab.scatter(ts, vs, color='g')
    pylab.axhline(v_e, color='g')
    pylab.axvline(t, color='k')
    xl = pylab.xlim()
    pylab.xlim(xl[0], xl[1]+0.5)
    yl = pylab.ylim()
    pylab.ylim(yl[0], yl[1]+0.5)
    pylab.title("Case %s: velocity vs time" % case)
    pylab.show()


def test(
        s=0.0, e=3.0, t=2.0, dt=0.1,
        v_s=0., v_e=1., a_s=0., a_e=0.,
        a_max=10.0, d_max=10.0, v_max=10.0,
        order=3):
    xs, ts, vs, case = compute_motion_profile(
        s, e, t, dt, v_s, v_e, a_s, a_e, a_max, d_max, v_max,
        order)
    #xs = numpy.cumsum(vs * dt)
    plot(ts, vs, xs, case, e, t, v_e)


def polynomial_3(s, e, v_s, v_e, t):
    a0 = s
    a1 = v_s
    a2 = (
        (-3 * (s - e) - (2 * v_s + v_e) * t) /
        (t * t))
    a3 = (
        (2 * (s - e) + (v_s + v_e) * t) /
        (t * t * t))
    return lambda dt: a0 + a1 * dt + a2 * dt ** 2. + a3 * dt ** 3.


def polynomial_5(s, e, v_s, v_e, a_s, a_e, t):
    a0 = s
    a1 = v_s
    a2 = a_s * 0.5
    a3 = (
        (20 * (e - s) - (8 * v_e + 12 * v_s) * t - (3 * a_e - a_s) * t ** 2.) /
        (2. * t ** 3.))
    a4 = (
        (
            -30 * (e - s) + (14 * v_e + 16 * v_s) * t +
            (3 * a_e - 2 * a_s) * t ** 2.) /
        (2. * t ** 4.))
    a5 = (
        (12 * (e - s) - 6 * (v_e + v_s) * t + (a_e - a_s) * t ** 2.) /
        (2 * t ** 5.))
    return lambda dt: (
        a0 + a1 * dt + a2 * dt ** 2. + a3 * dt ** 3. +
        a4 * dt ** 4. + a5 * dt ** 5.)


def compute_point_velocities(pts, ts, v_s, v_e):
    # TODO not sure if this works for vs[0] != 0 and vs[-1] != 0
    ss = [v_s]
    # first compute point-to-point slopes
    for i in xrange(1, len(pts) - 1):
        ss.append((pts[i] - pts[i - 1]) / (ts[i] - ts[i - 1]))
    ss.append(v_e)
    # then compute velocities
    vs = []
    for i in xrange(len(ss) - 1):
        if numpy.sign(ss[i]) == numpy.sign(ss[i + 1]):
            vs.append(0.5 * (ss[i] + ss[i + 1]))
        else:
            vs.append(0.)
    vs[0] = v_s
    vs.append(v_e)
    return vs


def compute_motion_profile(
        s=0.0, e=3.0, t=2.0, dt=0.1,
        v_s=0., v_e=1., a_s=0., a_e=0.,
        a_max=10.0, d_max=10.0, v_max=10.0,
        order=5):
    n = int(numpy.ceil(t / float(dt)))
    #d = abs(e - s)

    ts = numpy.linspace(0, t, n)
    if order == 3:
        p = polynomial_3(s, e, v_s, v_e, t)
    elif order == 5:
        p = polynomial_5(s, e, v_s, v_e, a_s, a_e, t)
    pts = [p(tp) for tp in ts]
    vs = compute_point_velocities(pts, ts, v_s, v_e)
    #vs[0] = v_s
    #vs[-1] = v_e
    return pts, ts, vs, -1


def compute_motion_profile_old(
        s=0.0, e=3.0, t=2.0, dt=0.1,
        v_s=0., v_e=1.,
        a_max=10.0, d_max=10.0, v_max=10.0):
    """
    Compute a motion profile

    Args:
    ------
        s : starting position
        e : ending position
        t : time to complete movement
        dt : time resolution of movement
        v_s : starting velocity
        v_e : ending velocity
        a_max : maximum acceleration
        d_max : maximum deceleration
        v_max : maximum velocity
    """
    n = int(numpy.ceil(t / float(dt)))
    d = abs(e - s)
    v_avg = d / t

    ts = numpy.linspace(0, t, n)
    case = None
    acc = abs(v_e - v_s) / t  # TODO cap acceleration
    if acc > a_max:
        raise Exception(
            "Movement requires acceleration[%s] over max [%s]"
            % (acc, a_max))

    # case 1: accel
    print("case 1?")
    ep = 0.5 * acc * t * t + v_s * t + s
    print("ep, e: %s, %s" % (ep, e))
    if approximate(ep, e):
        case = 1
        vs = numpy.linspace(v_s, v_e, n)
        return ts, vs, case

    if v_s == v_e and s != e:
        # accel then decel
        raise NotImplementedError
    # case 2: accel cruise
    print("case 2?")
    acc_t = (e - s - v_e * t) / (0.5 * abs(v_e - v_s) + v_s - v_e)
    print("acc_t: %s" % acc_t)
    if acc_t > 0.:
        acc = abs(v_e - v_s) / acc_t
        c_t = t - acc_t
        if c_t > 0:
            ep = c_t * v_e + 0.5 * acc * acc_t * acc_t + v_s * acc_t + s
            if c_t > 0 and approximate(ep, e):
                case = 2
                vs = numpy.ones_like(ts) * v_e
                am = ts < acc_t
                vs[am] = numpy.linspace(v_s, v_e, sum(am))
                return ts, vs, case

    # case 3: accel then decel, accel cruise then decel
    raise NotImplementedError


pylab.ion()
test(s=0.0, e=0.5, t=1.0, v_e=1.0, order=3)  # case 3
pylab.figure()
test(s=0.0, e=0.5, t=1.0, v_e=1.0, order=5)  # case 3
pylab.figure()
test(s=0.0, e=2.0, t=2.0, v_e=1.0, order=3)  # case 2
pylab.figure()
test(s=0.0, e=2.0, t=2.0, v_e=1.0, order=5)  # case 2
pylab.ioff()
pylab.show()
