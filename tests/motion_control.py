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
case 2
 - accel cruise
case 3
 - accel then decel
 - accel cruise then decel

don't enforce max accel
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
        s=0.0, e=3.0, t=2.0, dt=0.1, v_s=0., v_e=1.,
        a_max=10.0, d_max=10.0, v_max=10.0):
    ts, vs, case = compute_motion_profile(
        s, e, t, dt, v_s, v_e, a_max, d_max, v_max)
    xs = numpy.cumsum(vs * dt)
    plot(ts, vs, xs, case, e, t, v_e)

def compute_motion_profile(
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


#test(s=0.0, e=0.5, t=1.0, v_e=1.0)  # case 3
test(s=0.0, e=2.0, t=2.0, v_e=1.0)  # case 2
