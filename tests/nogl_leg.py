#!/usr/bin/env python
"""
"""

import sys

import numpy
from PyQt4 import (QtGui, QtCore)

import stompy


class Leg(object):
    def __init__(self, number=0):
        # joint angles
        self.hip = 0.
        self.thigh = 0.
        self.knee = 0.
        self.number = number

    def points(self):
        """Generate xyz points for links from angles"""
        return numpy.array(list(
            stompy.kinematics.leg.angles_to_points(
                self.hip, self.thigh, self.knee)))

    def _advance(self):
        """Fake leg movement"""
        if not hasattr(self, '_delta'):
            self._delta = ('hip', 0.05)
        if not hasattr(self, '_limits'):
            self._limits = stompy.geometry.get_limits(self.number)
        jn, da = self._delta
        jv = getattr(self, jn)
        nv = jv + da
        limits = self._limits[jn]
        if limits[0] <= nv <= limits[1]:
            setattr(self, jn, nv)
            return
        # no longer in limits
        if da > 0:
            jn = {'hip': 'thigh', 'thigh': 'knee', 'knee': 'hip'}[jn]
        da *= -1
        self._delta = (jn, da)
        return self._advance()


class OrthoProjection(object):
    # TODO clip elevation and azimuth
    # TODO sanitize scalar
    def __init__(self, elevation=0., azimuth=0., scalar=1., offset=None):
        # TODO use properties to cause update/repaint
        self.elevation = elevation
        self.azimuth = azimuth
        self.scalar = scalar
        if offset is None:
            offset = (0, 0)
        self.offset = offset

    def to_transform(self):
        # rotate points by elevation and azimuth
        # apply yaw first
        yT = stompy.transforms.rotation_3d(0., 0., self.azimuth)
        eT = stompy.transforms.rotation_3d(self.elevation, 0., 0.)
        #return yT * eT
        return eT * yT

    def project_points(self, pts):
        """Project a set of xyz points to xy"""
        # TODO cache?
        T = self.to_transform()
        tpts = stompy.transforms.transform_3d_array(T, pts)
        #print(tpts)
        # then apply scaling and throw out z
        spts = tpts[:, :2] * self.scalar
        # apply offset
        #print(spts)
        opts = spts + numpy.array(self.offset)[numpy.newaxis, :]
        #print(opts)
        return opts


class LegDisplay(QtGui.QWidget):
    def __init__(self):
        super(LegDisplay, self).__init__()
        self.projection = OrthoProjection()
        self.leg = Leg()
        self._pens = {
            'links': [
                QtGui.QPen(QtCore.Qt.red, 2),  # hip link
                QtGui.QPen(QtCore.Qt.green, 2),  # thigh link
                QtGui.QPen(QtCore.Qt.blue, 2),  # calf link
            ],
            'axes': [
                QtGui.QPen(QtCore.Qt.red, 1, QtCore.Qt.DotLine),
                QtGui.QPen(QtCore.Qt.green, 1, QtCore.Qt.DotLine),
                QtGui.QPen(QtCore.Qt.blue, 1, QtCore.Qt.DotLine),
            ],
            'limits': QtGui.QPen(QtCore.Qt.magenta, 1, QtCore.Qt.DashLine),
        }

    def resizeEvent(self, event):
        # account for any user applied offset
        os = event.oldSize()
        ow, oh = os.width(), os.height()
        ohw, ohh = ow / 2, oh / 2
        dx = self.projection.offset[0] - ohw
        dy = self.projection.offset[1] - ohh
        # event.oldSize()
        self.projection.offset = (
            self.width() / 2 + dx,
            self.height() / 2 + dy)

    def paintEvent(self, event):
        print(
            "Azimuth: %s, Elevation: %s" %
            (self.projection.azimuth, self.projection.elevation))
        #self.projection.offset = (self.width() / 2, self.height() / 2)

        painter = QtGui.QPainter()
        painter.begin(self)

        # apply transform to center 0,0
        #T = painter.worldTransform()
        #T.translate(self.width() / 2, self.height() / 2)
        #painter.setWorldTransform(T, combine=False)

        painter.setRenderHints(QtGui.QPainter.Antialiasing)

        # draw axes
        sx, sy = self.projection.offset
        #sx, sy = 0, 0
        for (i, pen) in enumerate(self._pens['axes']):
            pt = [0., 0., 0.]
            pt[i] = 12.
            x, y = self.projection.project_points([pt])[0]
            painter.setPen(pen)
            painter.drawLine(sx, sy, x, y)

        # draw leg
        pts = self.leg.points()
        links = self.projection.project_points(pts)
        sx, sy = self.projection.offset
        #sx, sy = 0, 0
        for (pen, link) in zip(self._pens['links'], links):
            painter.setPen(pen)
            x, y = link
            painter.drawLine(sx, sy, x, y)
            sx, sy = x, y

        # draw limits
        z = pts[-1][2]
        lpts = stompy.kinematics.leg.limits_at_z_3d(
            z, self.leg.number)
        if lpts is not None:
            tpts = self.projection.project_points(lpts)
            pen = self._pens['limits']
            painter.setPen(self._pens['limits'])
            painter.drawPolyline(
                QtGui.QPolygonF([QtCore.QPointF(*pt) for pt in tpts]))

        # TODO draw foot

        # TODO draw load

        painter.end()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            # TODO bring out reset positions
            self.projection.azimuth = 0
            self.projection.elevation = numpy.pi / 2.
            self.update()
        if event.button() == QtCore.Qt.MiddleButton:
            pt = event.pos()
            self._middle_click_pos = pt.x(), pt.y()
        if event.button() == QtCore.Qt.LeftButton:
            # right mouse clicked
            pt = event.pos()
            self._right_click_pos = pt.x(), pt.y()

    def mouseMoveEvent(self, event):
        if hasattr(self, '_right_click_pos'):
            # drag
            x0, y0 = self._right_click_pos
            pt = event.pos()
            x1, y1 = pt.x(), pt.y()
            dx = x1 - x0
            dy = y1 - y0
            update = False
            # TODO bring out rotation scaling factors
            if (dx != 0):  # change azimuth
                self.projection.azimuth -= dx * 0.01
                update = True
            if (dy != 0):  # change elevation
                self.projection.elevation -= dy * 0.01
                update = True
            if update:
                self.update()
            self._right_click_pos = (x1, y1)
        if hasattr(self, '_middle_click_pos'):
            # drag
            x0, y0 = self._middle_click_pos
            pt = event.pos()
            x1, y1 = pt.x(), pt.y()
            dx = x1 - x0
            dy = y1 - y0
            self.projection.offset = (
                self.projection.offset[0] + dx,
                self.projection.offset[1] + dy)
            self.update()
            self._middle_click_pos = (x1, y1)

    def mouseReleaseEvent(self, event):
        if hasattr(self, '_right_click_pos'):
            del self._right_click_pos
        if hasattr(self, '_middle_click_pos'):
            del self._middle_click_pos

    def wheelEvent(self, event):
        d = event.delta()
        print(d)
        nd = max(-1., min(1., d / 1000.)) + 1.
        self.projection.scalar *= nd
        self.update()


def main():
    app = QtGui.QApplication(sys.argv)
    #w = QtGui.QWidget()
    w = LegDisplay()
    w.resize(300, 300)
    w.move(300, 300)
    w.setWindowTitle('Simple')
    w.show()
    w.projection.scalar = 3.
    w.projection.elevation = numpy.pi / 4.

    def tick():
        #w.projection.azimuth += numpy.pi / 50.
        #w.projection.elevation += numpy.pi / 50.

        # fake move leg
        w.leg._advance()

        w.update()

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
