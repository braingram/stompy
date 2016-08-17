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
 - accel then decel
 - accel cruise then decel
case 2
 - accel cruise
case 3
 - accel

don't enforce max accel
"""

import numpy
import pylab


s = 0.0  # meters, starting position
e = 0.5  # meters, ending position
t = 1.0  # seconds, time to complete movement
dt = 0.1  # seconds
n = int(numpy.ceil(t / float(dt)))

a = 10.0  # m / s ** 2, acceleration
d = 10.0  # m / s ** 2, deceleration
#j = 100.0  # m / s ** 3, jerk

v_s = 0.  # m / s, starting velocity
v_e = 1.  # m / s, ending velocity

v_m = 10.0  # m / s, maximum

d = abs(e - s)  # m, distance traveled
v_avg = d / t  # m / s, average velocity

approximate = lambda v, t, e=0.001: abs(v - t) < e

# case 3: accel
acc = (v_e - v_s) / t
ep = 0.5 * acc * t * t + v_s * t + s
print(acc, ep)
if approximate(ep, e):
    ts = numpy.linspace(0, t, n)
    vs = numpy.linspace(v_s, v_e, n)
    xs = numpy.cumsum(vs * dt)

# from: firmware/tinyg/plan_line.c[161]
#t_a = 2 * numpy.sqrt(abs(v_s - v_c) / jerk)
#t_d = 2 * numpy.sqrt(abs(v_c - v_e) / jerk)

#t_a = abs(v_c - v_s) / a  # s, acceleration time
#t_d = abs(v_c - v_e) / d  # s, deceleration time


pylab.subplot(211)
pylab.plot(ts, xs, color='b')
pylab.scatter(ts, xs, color='b')
pylab.axhline(e, color='b')
pylab.axvline(t, color='k')
xl = pylab.xlim()
pylab.xlim(xl[0], xl[1]+0.5)
yl = pylab.ylim()
pylab.ylim(yl[0], yl[1]+0.5)
pylab.title("positon vs time")
pylab.subplot(212)
pylab.plot(ts, vs, color='g')
pylab.scatter(ts, vs, color='g')
pylab.axhline(v_e, color='g')
pylab.axvline(t, color='k')
xl = pylab.xlim()
pylab.xlim(xl[0], xl[1]+0.5)
yl = pylab.ylim()
pylab.ylim(yl[0], yl[1]+0.5)
pylab.title("velocity vs time")
pylab.show()
