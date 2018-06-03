#!/usr/bin/env python
"""
at far distance (1000000) angle is very small

Draw origin and lines along x, y and z
Draw sphere:
    - center
    - line from center to 0,0
    - line frm center to endpoint of arc
Draw path along sphere
    - draw arc

when all the way to the left or right, (1 or -1) radius should be 0
when at 0, radius should be large (1M)
maybe 0.1 in = 1/2 * 1M
0.2 = 1/4 * 1M
0.3 = 1/8 * 1M
...
1000000 / 2 ** (v * 10.)
"""

import pyqtgraph
import pyqtgraph.opengl
import numpy

import stompy


thumb_mid = 127.5
thumb_db = 12.5  # +-
thumb_scale = (255 - thumb_mid) - thumb_db

max_radius = 100000.

joy = stompy.joystick.ps3.PS3Joystick()
joy_limits = {}

app = pyqtgraph.Qt.QtGui.QApplication([])
win = pyqtgraph.opengl.GLViewWidget()
#w.opts['distance'] = 20
win.show()

#g = gl.GLGridItem()
#w.addItem(g)

scene = {}
# -- add items --
# origin: (lines)
w = 1
tc = lambda s: pyqtgraph.mkColor(s).getRgb()
scene['origin'] = {
    'x': pyqtgraph.opengl.GLLinePlotItem(
        pos=numpy.array([[0, 0, 0], [1, 0, 0]]),
        color=tc('r'), width=w, antialias=True),
    'y': pyqtgraph.opengl.GLLinePlotItem(
        pos=numpy.array([[0, 0, 0], [0, 1, 0]]),
        color=tc('g'), width=w, antialias=True),
    'z': pyqtgraph.opengl.GLLinePlotItem(
        pos=numpy.array([[0, 0, 0], [0, 0, 1]]),
        color=tc('b'), width=w, antialias=True),
}
# start: line from center to 0, 0
scene['start'] = pyqtgraph.opengl.GLLinePlotItem(
    pos=numpy.array([[-1, 0, 0], [0, 0, 0]]),
    color=tc('y'), width=2, antialias=True)
# end: line from center to end of arc
scene['end'] = pyqtgraph.opengl.GLLinePlotItem(
    pos=numpy.array([[-1, 0, 0], [0, -1, 0]]),
    color=tc('m'), width=2, antialias=True)
# arc: line (maybe several straight lines?)
scene['arc'] = pyqtgraph.opengl.GLLinePlotItem(
    pos=numpy.array([[0, 0, 0], [0, 0, -1]]),
    color=tc('c'), width=2, antialias=True)

for k in scene:
    if isinstance(scene[k], dict):
        for sk in scene[k]:
            win.addItem(scene[k][sk])
    else:
        win.addItem(scene[k])


def read_axis(k):
    a = joy.smoothed_axes.get(k, None)
    if a is None:
        v = 0.
    else:
        v = a.update() - thumb_mid
    #print(k, v, joy.axes.get(k, None))
    #v = joy.smoothed_axes.get(k, thumb_mid) - thumb_mid
    if abs(v) < thumb_db:
        return 0.
    return numpy.sign(v) * min(1.0, (abs(v) - thumb_db) / float(thumb_scale))


def update():
    joy.update(max_time=0.05)

    #rt = lambda s: (
    #    (joy.axes.get(s, thumb_mid) - thumb_mid) / float(thumb_scale))
    ax = -read_axis('thumb_right_x')
    ay = -read_axis('thumb_left_y')
    aa = read_axis('thumb_left_x')
    ab = -read_axis('thumb_right_y')

    pl = joy.keys.get('one_right', False)
    for k in (
            'thumb_left_x', 'thumb_left_y', 'thumb_right_x', 'thumb_right_y'):
        v = joy.smoothed_axes.get(k, None)
        if v is None:
            continue
        v = v.update()
        if k not in joy_limits:
            joy_limits[k] = {'min': v, 'max': v}
        else:
            joy_limits[k]['min'] = min(joy_limits[k]['min'], v)
            joy_limits[k]['max'] = max(joy_limits[k]['max'], v)
        if pl:
            print("%s[%s]: %s, %s" % (
                k, v, joy_limits[k]['min'], joy_limits[k]['max']))
    if pl:
        print(joy_limits)
    # - up (y) moves forward: 0 -> 0
    # - left (x) changes radius: 0 -> large radius
    # -1, 0, 1
    #  0
    #radius = numpy.sign(ax) * (1000000. - 1000000. * numpy.abs(ax))
    aax = numpy.abs(ax)
    if numpy.abs(ax) < 0.001:  # sign(0.0) == 0.
        radius = max_radius
    else:
        #radius = numpy.sign(ax) * numpy.log(aax) * 500.
        #radius = numpy.sign(ax) * (2 + ((1 - aax) ** 50.) * 100000.)
        radius = (
            numpy.sign(ax) * max_radius / 2. ** (numpy.log2(max_radius) * aax))
    #speed = numpy.max([abs(v) for v in [ax, ay, aa, ab]])
    speed = ay * 3.
    if abs(radius) < 0.1:
        rspeed = 0.
    else:
        rspeed = speed / radius
    if pl:
        print("ax:", ax)
        print("speed:", speed)
        print("radius:", radius)
        print("rspeed:", rspeed)

    # can I crab walk and turn at the same time?
    # - z could move center of rotation rostral or caudal
    #cx, cy, cz = (-1000000, 0, 0)
    cx, cy, cz = (radius, 0, 0)
    rx, ry, rz = (0, 0, rspeed)

    T = stompy.transforms.rotation_about_point_3d(
        cx, cy, cz, rx, ry, rz)
    npts = 10
    rs = 1.0 / npts
    sT = stompy.transforms.rotation_about_point_3d(
        cx, cy, cz, rx * rs, ry * rs, rz * rs)
    tx, ty, tz = stompy.transforms.transform_3d(T, 0, 0, 0)
    # re-compute target
    # -- update --
    scene['start'].setData(pos=numpy.array([[cx, cy, cz], [0, 0, 0]]))
    scene['end'].setData(pos=numpy.array([[0, 0, 0], [tx, ty, tz]]))
    # TODO break up arc
    p = [0, 0, 0]
    pts = []
    for _ in xrange(npts):
        pts.append(p)
        p = stompy.transforms.transform_3d(sT, *p)
    pts.append(p)
    scene['arc'].setData(pos=numpy.array(pts))


t = pyqtgraph.Qt.QtCore.QTimer()
t.timeout.connect(update)
t.start(50)

if __name__ == '__main__':
    pyqtgraph.QtGui.QApplication.instance().exec_()
