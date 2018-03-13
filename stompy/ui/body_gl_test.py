# -*- coding: utf-8 -*-
"""
Demonstrate use of GLLinePlotItem to draw cross-sections of a surface.

"""
## Add path to library (just for examples; you do not need this)

import PIL.Image
import PIL.ImageDraw
import PIL.ImageFont


from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy

import stompy

side_view = {
    'center': QtGui.QVector3D(60, 0, 0),
    'azimuth': 90,
    'elevation': 0,
    # 'distance': 150,
    'distance': 9000,
    'fov': 1,
}

top_view = side_view.copy()
top_view['elevation'] = 90

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


def draw_text(txt):
    im = PIL.Image.fromarray(
        numpy.zeros((100, 300, 4), numpy.uint8))
    draw = PIL.ImageDraw.Draw(im)
    font = PIL.ImageFont.truetype(
        "/usr/share/fonts/truetype/freefont/FreeMono.ttf", 80)
    draw.text((10, 10), txt, (255, 255, 255, 255), font=font)
    return numpy.array(im, numpy.uint8)


tim = draw_text('test')
gtxt = gl.GLImageItem(tim)
gtxt.scale(0.1, 0.1, 0.1)
gtxt.rotate(90, 0, 0, 1)
w.addItem(gtxt)


def plot_legs(angles, items=None):
    make_new = False
    if items is None:
        items = {}
        make_new = True
    for leg in angles:
        h, t, k, l = angles[leg]
        pts = [[0., 0., 0.], ] + list(
            stompy.kinematics.leg.angles_to_points(h, t, k))
        pts = stompy.kinematics.body.leg_to_body_array(leg, pts)
        if make_new:
            items[leg] = {}

            items[leg]['hip'] = gl.GLLinePlotItem(
                pos=numpy.array([pts[0], pts[1]]), color=[1., 0., 0., 1.],
                width=5, antialias=True)
            items[leg]['thigh'] = gl.GLLinePlotItem(
                pos=numpy.array([pts[1], pts[2]]), color=[0., 1., 0., 1.],
                width=5, antialias=True)
            items[leg]['knee'] = gl.GLLinePlotItem(
                pos=numpy.array([pts[2], pts[3]]), color=[0., 0., 1., 1.],
                width=5, antialias=True)
            items[leg]['load'] = gl.GLScatterPlotItem(
                pos=pts[3], color=[1., 0.5, 0., 1.], size=l/50., pxMode=True)
            w.addItem(items[leg]['hip'])
            w.addItem(items[leg]['thigh'])
            w.addItem(items[leg]['knee'])
            w.addItem(items[leg]['load'])
        else:
            items[leg]['hip'].setData(pos=numpy.array([pts[0], pts[1]]))
            items[leg]['thigh'].setData(pos=numpy.array([pts[1], pts[2]]))
            items[leg]['knee'].setData(pos=numpy.array([pts[2], pts[3]]))
            items[leg]['load'].setData(pos=pts[3], size=l/50.)
    return items


angles = {}
deltas = {}
limits = [[0.6, -0.6], [1.1, 0.], [0., -1.1], [2000, 0]]
for leg in stompy.kinematics.body.leg_to_body_transforms.keys():
    angles[leg] = []
    for l in limits:
        a = numpy.random.random() * (l[1] - l[0]) + l[0]
        angles[leg].append(a)
    angles[leg] = numpy.array(angles[leg])
    deltas[leg] = numpy.array([0.01, 0.01, -0.02, 50.])
items = plot_legs(angles)


def update(angles=angles, items=items, deltas=deltas):
    plot_legs(angles, items=items)
    for leg in angles:
        angles[leg] += deltas[leg]
        for i in xrange(len(limits)):
            if angles[leg][i] > limits[i][0] or angles[leg][i] < limits[i][1]:
                deltas[leg][i] *= -1
                print(angles[leg], limits, deltas[leg])

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
#timer = QtCore.QTimer()
#timer.timeout.connect(update2)
#timer.start(5000)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    pass
