# -*- coding: utf-8 -*-
"""
Demonstrate use of GLLinePlotItem to draw cross-sections of a surface.

"""
## Add path to library (just for examples; you do not need this)

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy

side_view = {
    'center': QtGui.QVector3D(45, 0, 0),
    'azimuth': 90,
    'elevation': 0,
    'distance': 150,
}

top_view = side_view.copy()
top_view['elevation'] = 90


HIP_LENGTH = 11
THIGH_LENGTH = 54
KNEE_LENGTH = 72
THIGH_REST_ANGLE = 1.4663392033212757
KNEE_REST_ANGLE = -1.4485440219526171

app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.opts['distance'] = 40
w.show()
w.setWindowTitle('Stompy leg display test')

gi = 120
gx = gl.GLGridItem()
gx.setSize(gi, gi, 1)
gx.setSpacing(6, 6, 1)
gx.rotate(90, 0, 1, 0)
#gx.translate(-25, 0, 0)
w.addItem(gx)
gy = gl.GLGridItem()
gy.setSize(gi, gi, 1)
gy.setSpacing(6, 6, 1)
gy.rotate(90, 1, 0, 0)
gy.translate(gi / 2, 0, 0)
w.addItem(gy)
gz = gl.GLGridItem()
gz.setSize(gi, gi, 1)
gz.setSpacing(6, 6, 1)
gz.translate(gi / 2, 0, 0)
w.addItem(gz)


def generate_leg_points(hip, thigh, knee):
    x = HIP_LENGTH
    z = 0
    ch = numpy.cos(hip)
    sh = numpy.sin(hip)
    yield x * ch, x * sh, z

    a = THIGH_REST_ANGLE - thigh
    x += THIGH_LENGTH * numpy.cos(a)
    z += THIGH_LENGTH * numpy.sin(a)
    yield x * ch, x * sh, z

    a = KNEE_REST_ANGLE - knee - thigh
    x += KNEE_LENGTH * numpy.cos(a)
    z += KNEE_LENGTH * numpy.sin(a)

    yield x * ch, x * sh, z


def plot_leg(hip, thigh, knee, items=None):
    pts = list(generate_leg_points(hip, thigh, knee))
    if items is None:
        items = {}
        items['hip'] = gl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], pts[0]]), color=[1., 0., 0., 1.],
            width=5, antialias=True)
        items['thigh'] = gl.GLLinePlotItem(
            pos=numpy.array([pts[0], pts[1]]), color=[0., 1., 0., 1.],
            width=5, antialias=True)
        items['knee'] = gl.GLLinePlotItem(
            pos=numpy.array([pts[1], pts[2]]), color=[0., 0., 1., 1.],
            width=5, antialias=True)
        w.addItem(items['hip'])
        w.addItem(items['thigh'])
        w.addItem(items['knee'])
    else:
        items['hip'].setData(pos=numpy.array([[0, 0, 0], pts[0]]))
        items['thigh'].setData(pos=numpy.array([pts[0], pts[1]]))
        items['knee'].setData(pos=numpy.array([pts[1], pts[2]]))
    return items


angles = [0, 0, 0]
deltas = [0.01, 0.01, -0.02]
items = plot_leg(*angles)


def update(angles=angles, items=items, deltas=deltas):
    plot_leg(angles[0], angles[1], angles[2], items=items)
    angles[0] += deltas[0]
    if angles[0] > 0.6 or angles[0] < -0.6:
        deltas[0] *= -1
    angles[1] += deltas[1]
    if angles[1] > 1.1 or angles[1] < 0.:
        deltas[1] *= -1
    angles[2] += deltas[2]
    if angles[2] < -1.1 or angles[2] > 0:
        deltas[2] *= -1

state = [0, ]


def update2(state=state):
    if state[0] == 0:
        w.opts.update(side_view)
        w.update()
    elif state[0] == 1:
        w.opts.update(top_view)
        w.update()
    if state[0] < 1:
        state[0] += 1
    else:
        state[0] = 0


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)
timer = QtCore.QTimer()
timer.timeout.connect(update2)
timer.start(5000)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    pass
