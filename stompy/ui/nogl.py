#!/usr/bin/env python
"""
"""

import sys
import time

import numpy

from PyQt5 import (QtGui, QtCore, QtWidgets)
from PyQt5.QtWidgets import (
    QGestureEvent, QPinchGesture,
    QSwipeGesture,
    QWidget, QApplication)

from .. import kinematics
#from .. import geometry
from .. import transforms


class Leg(object):
    def __init__(self, number=0):
        # joint angles
        self.hip = 0.
        self.thigh = 0.
        self.knee = 0.
        self.calf = 0.
        self.number = number
        # TODO make this not a hack
        self.geometry = kinematics.leg.LegGeometry(self.number)
        self.restriction = {}

    def set_angles(self, hip, thigh, knee, calf=0.):
        self.hip = hip
        self.thigh = thigh
        self.knee = knee
        self.calf = calf

    def points(self):
        """Generate xyz points for links from angles"""
        return numpy.array(list(
            self.geometry.angles_to_points(
                self.hip, self.thigh, self.knee)))

    def _advance(self):
        """Fake leg movement"""
        if not hasattr(self, '_delta'):
            self._delta = ('hip', 0.05)
        if not hasattr(self, '_limits'):
            self._limits = self.geometry.get_limits()
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
        yT = transforms.rotation_3d(0., 0., self.azimuth)
        eT = transforms.rotation_3d(self.elevation, 0., 0.)
        #return yT * eT
        return eT * yT

    def get_offset(self):
        return numpy.array(self.offset) * numpy.array(self.window_size)

    def project_points(self, pts):
        """Project a set of xyz points to xy"""
        # TODO cache?
        T = self.to_transform()
        tpts = transforms.transform_3d_array(T, pts)
        # then apply scaling and throw out z
        spts = tpts[:, :2] * self.scalar
        spts[:, 0] *= -1  # swap to right hand coords
        # apply offset
        opts = spts + numpy.array(self.get_offset())[numpy.newaxis, :]
        return opts


class LegDisplay(QWidget):
    views = {
        'right': {
            'azimuth': numpy.pi,
            'elevation': numpy.pi / 2,
            'offset': (0.25, 0.5),
            'scalar': 3.,
        },
        'left': {
            'azimuth': -numpy.pi,
            'elevation': numpy.pi / 2,
            'offset': (0.25, 0.5),
            'scalar': 3.,
        },
        'top': {
            'azimuth': 0,
            'elevation': 0,
            'offset': (0.5, 0.5),
            'scalar': 3.,
        },
        # TODO front
        # TODO back
    }

    def __init__(self, parent=None):
        super(LegDisplay, self).__init__(parent)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)
        for g in (
                QtCore.Qt.PanGesture, QtCore.Qt.SwipeGesture,
                QtCore.Qt.PinchGesture):
            self.grabGesture(g)
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
            'calf': QtGui.QPen(QtCore.Qt.cyan, 1),
            'support': QtGui.QPen(QtCore.Qt.darkCyan, 1),
            'COG': QtGui.QPen(QtCore.Qt.darkRed, 1),
            'path': QtGui.QPen(QtCore.Qt.black, 1),
        }
        self.paintsPerSecond = 10.
        self._last_update_time = time.time() - 1.

    def set_view(self, view):
        if not isinstance(view, dict):
            view = self.views[view]
        for k in view:
            #print("old: %s" % (getattr(self.projection, k),))
            setattr(self.projection, k, view[k])
            #print("new: %s" % (getattr(self.projection, k),))
        self.update()

    def event(self, event):
        if isinstance(event, QGestureEvent):
            return self.gestureEvent(event)
        return super(LegDisplay, self).event(event)

    def gestureEvent(self, event):
        r = True
        for g in event.gestures():
            if isinstance(g, QPinchGesture):
                self.projection.scalar *= g.scaleFactor()
                self.update()
                r = False
            elif isinstance(g, QSwipeGesture):
                pass
                #d = g.verticalDirection()
                #if d == QSwipeGesture.NoDirection:
                #    d = g.horizontalDirection()
                #if d == QSwipeGesture.Down:
                #    self.set_view('top')
                #elif d == QSwipeGesture.Left:
                #    self.set_view('right')
                #elif d == QSwipeGesture.Right:
                #    self.set_view('left')
                r = False
            else:
                print("Gesture event: %s" % (g, ))
        return r

    def update(self, *args, **kwargs):
        t = time.time()
        dt = t - self._last_update_time
        if dt > (1. / self.paintsPerSecond):
            super(LegDisplay, self).update(*args, **kwargs)
            self._last_update_time = t

    def resizeEvent(self, event):
        self.projection.window_size = (self.width(), self.height())
        return
        # account for any user applied offset
        os = event.oldSize()
        if os.width() == -1:
            return
        dx = self.projection.offset[0] - os.width() / 2
        dy = self.projection.offset[1] - os.height() / 2
        #print(os.width(), os.height(), self.width(), self.height(), dx, dy)
        # event.oldSize()
        self.projection.offset = (
            self.width() / 2 + dx,
            self.height() / 2 + dy)

    def _paintAxes(self, painter):
        # draw axes
        sx, sy = self.projection.get_offset()
        #sx, sy = 0, 0
        for (i, pen) in enumerate(self._pens['axes']):
            pt = [0., 0., 0.]
            pt[i] = 12.
            x, y = self.projection.project_points([pt])[0]
            painter.setPen(pen)
            painter.drawLine(sx, sy, x, y)

    def _paintLeg(self, painter, leg=None, transform=None):
        if leg is None:
            leg = self.leg
        saved_brush = painter.brush()

        pts = [[0, 0, 0], ] + list(leg.points())
        if transform is not None:
            pts = transforms.transform_3d_array(transform, pts)
        links = self.projection.project_points(pts)
        #sx, sy = self.projection.offset
        sx, sy = links[0]
        #sx, sy = 0, 0
        for (pen, link) in zip(self._pens['links'], links[1:]):
            painter.setPen(pen)
            x, y = link
            painter.drawLine(sx, sy, x, y)
            sx, sy = x, y

        # draw calf load (circle at sx, sy)
        r = leg.calf / 50.
        if r > 1:
            painter.setPen(self._pens['calf'])
            #painter.setBrush(QtGui.QBrush(self._pens['calf'].color()))
            painter.drawEllipse(x - r, y - r, r * 2, r * 2)

        # draw limits
        z = pts[-1][2]
        lpts = leg.geometry.limits_at_z_3d(z)
        # color limits by restriction
        r = max(0., min(1., leg.restriction.get('r', 1.)))
        pen = QtGui.QPen(QtGui.QColor(
            255 * r, (1 - r) * 255, 0.), 2,
            QtCore.Qt.DashLine)
        painter.setPen(pen)
        if leg.number == getattr(self, 'selected_leg', None):
            brush = QtGui.QBrush(QtGui.QColor(
                128, 0, 0, 32))
            df = painter.drawPolygon
            painter.setBrush(brush)
        else:
            df = painter.drawPolyline
        if lpts is not None:
            if transform is not None:
                lpts = transforms.transform_3d_array(transform, lpts)
            tpts = self.projection.project_points(lpts)
            #painter.drawPolyline(
            #    QtGui.QPolygonF([QtCore.QPointF(*pt) for pt in tpts]))
            #painter.drawPolygon(
            #    QtGui.QPolygonF([QtCore.QPointF(*pt) for pt in tpts]))
            df(QtGui.QPolygonF([QtCore.QPointF(*pt) for pt in tpts]))
        #painter.setBrush(QtGui.QBrush(None))
        painter.setBrush(saved_brush)
        return x, y

    def reportTiming(self, nseconds=1.):
        t = time.time()
        if not hasattr(self, '_last_report_time'):
            self._last_report_time = t
            self._reports = 1
            return
        self._reports += 1
        dt = t - self._last_report_time
        if dt > nseconds:
            print(
                "%s: %s calls in %s seconds" %
                (self.__class__.__name__, self._reports, dt))
            self._last_report_time = t
            self._reports = 0

    def paintEvent(self, event):
        #self.reportTiming()
        #print(
        #    "Azimuth: %s, Elevation: %s" %
        #    (self.projection.azimuth, self.projection.elevation))
        #self.projection.offset = (self.width() / 2, self.height() / 2)

        painter = QtGui.QPainter()
        painter.begin(self)

        # apply transform to center 0,0
        #T = painter.worldTransform()
        #T.translate(self.width() / 2, self.height() / 2)
        #painter.setWorldTransform(T, combine=False)

        painter.setRenderHints(QtGui.QPainter.Antialiasing)

        # draw axes
        self._paintAxes(painter)

        # draw leg
        self._paintLeg(painter)

        # TODO draw foot

        # TODO draw load

        painter.end()

    def keyPressEvent(self, event):
        #print("keyPressEvent: %s" % (event.key(), ))
        k = event.key()
        if k == QtCore.Qt.Key_T:
            self.set_view('top')
        elif k == QtCore.Qt.Key_L:
            self.set_view('left')
        elif k == QtCore.Qt.Key_R:
            self.set_view('right')
        elif k == QtCore.Qt.Key_F:
            self.set_view('front')
        elif k == QtCore.Qt.Key_B:
            self.set_view('back')
        return super(LegDisplay, self).keyPressEvent(event)

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
                self.projection.azimuth += dx * 0.01
                update = True
            if (dy != 0):  # change elevation
                self.projection.elevation -= dy * 0.01
                update = True
            if update:
                self.update()
            #print(self.projection.azimuth, self.projection.elevation)
            self._right_click_pos = (x1, y1)
        if hasattr(self, '_middle_click_pos'):
            # drag
            x0, y0 = self._middle_click_pos
            pt = event.pos()
            x1, y1 = pt.x(), pt.y()
            dx = (x1 - x0) / float(self.width())
            dy = (y1 - y0) / float(self.height())
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
        nd = max(-1., min(1., d / 1000.)) + 1.
        self.projection.scalar *= nd
        self.update()


class BodyDisplay(LegDisplay):
    views = {
        'left': {
            'azimuth': -numpy.pi / 2.,
            'elevation': numpy.pi / 2.,
            'offset': (0.5, 0.5),
            'scalar': 1.,
        },
        'right': {
            'azimuth': numpy.pi / 2.,
            'elevation': numpy.pi / 2.,
            'offset': (0.5, 0.5),
            'scalar': 1.,
        },
        'front': {
            'azimuth': 0,
            'elevation': numpy.pi / 2.,
            'offset': (0.5, 0.5),
            'scalar': 1.,
        },
        'back': {
            'azimuth': numpy.pi,
            'elevation': numpy.pi / 2.,
            'offset': (0.5, 0.5),
            'scalar': 1.,
        },
        'top': {
            'azimuth': numpy.pi,
            'elevation': 0,
            'offset': (0.5, 0.5),
            'scalar': 1.,
        },
    }

    def __init__(self, parent=None):
        super(BodyDisplay, self).__init__(parent)
        del self.leg
        self.legs = {}
        self.support_legs = []
        self.path = []
        self.COG = [0, 0, 0]

    def add_leg(self, leg_number):
        self.legs[leg_number] = Leg(leg_number)

    def paintEvent(self, event):
        #self.reportTiming()
        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setRenderHints(QtGui.QPainter.Antialiasing)
        self._paintAxes(painter)
        supports = []
        for leg in sorted(self.legs):
            T = kinematics.body.leg_to_body_transforms[leg]
            xy = self._paintLeg(painter, self.legs[leg], T)
            if leg in self.support_legs:
                supports.append(xy)
        if len(supports):
            pen = self._pens['support']
            # project points
            supports += [supports[0], ]
            for p0, p1 in zip(supports[:-1], supports[1:]):
                painter.setPen(pen)
                painter.drawLine(p0[0], p0[1], p1[0], p1[1])
        if len(self.path):
            pen = self._pens['path']
            xys = self.projection.project_points(self.path)
            for p0, p1 in zip(xys[:-1], xys[1:]):
                painter.setPen(pen)
                painter.drawLine(p0[0], p0[1], p1[0], p1[1])
        if self.COG is not None:
            pen = self._pens['COG']
            xyz = self.projection.project_points([[
                self.COG[0], self.COG[1], 0.]])[0]
            painter.setPen(pen)
            painter.setBrush(QtGui.QBrush(self._pens['COG'].color()))
            r = 3
            d = r * 2
            painter.drawEllipse(xyz[0]-r, xyz[1]-r, d, d)
        painter.end()


class ChartData(object):
    max_n = 200

    def __init__(self, label=''):
        self.data = []
        self.label = ''

    def append(self, value):
        self.data.append(value)
        if len(self.data) > self.max_n:
            self.data = self.data[-self.max_n:]

    def get_data(self):
        return self.data

    def clear(self):
        self.data = []


class LineChart(QWidget):
    _pens = [
        QtGui.QPen(QtCore.Qt.blue, 1),
        QtGui.QPen(QtCore.Qt.green, 1),
        QtGui.QPen(QtCore.Qt.red, 1),
    ]

    _cfg = {
        'axis_width': 20,
        'axis_text_width': 50,
        'margin': 5,
    }

    def __init__(self, parent=None):
        self.data = {}
        self._pen_index = 0
        super(LineChart, self).__init__(parent)

    def addSeries(self, label, pen=None):
        self.data[label] = ChartData(label=label)
        if pen is None:
            pen = self._pens[self._pen_index]
            self._pen_index = (self._pen_index + 1) % len(self._pens)
        self.data[label]._pen = pen

    def appendData(self, label, value):
        self.data[label].append(value)

    def clearData(self, label=None):
        if label is None:
            return [self.clearData(l) for l in self.data]
        self.data[label].clear()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setRenderHints(QtGui.QPainter.Antialiasing)
        ww, wh = self.width(), self.height()
        n_series = len(self.data)
        # give each axis 20 pixels
        aw = self._cfg['axis_width']
        margin = self._cfg['margin']
        axis_margin = n_series * aw
        # give space for axes at left
        xs = (
            numpy.arange(ChartData.max_n) *
            (ww - axis_margin - margin) / float(ChartData.max_n) + axis_margin)
        for (series_i, label) in enumerate(self.data):
            # draw axis
            ax = series_i * aw
            painter.setPen(self.data[label]._pen)
            painter.drawLine(
                ax + aw - 2, margin, ax + aw - 2, wh - margin)
            # min
            painter.drawLine(
                ax + aw - 8, margin, ax + aw - 2, margin)
            # max
            painter.drawLine(
                ax + aw - 8, wh - margin, ax + aw - 2, wh - margin)
            # mid
            painter.drawLine(
                ax + aw - 8, wh / 2, ax + aw - 2, wh / 2)
            # plot data
            vs = numpy.array(self.data[label].get_data())
            if len(vs) == 0:
                continue
            minv, maxv = numpy.min(vs), numpy.max(vs)

            painter.setFont(QtGui.QFont('Arial', 10, 1))

            axis_tw = self._cfg['axis_text_width']
            painter.save()
            painter.translate(ax + 2, wh - margin * 2)
            painter.rotate(-90)
            painter.drawText(
                0, 0, axis_tw, aw,
                QtCore.Qt.AlignLeft, '%i' % int(minv))
            painter.restore()

            painter.save()
            painter.translate(ax + 2, margin * 2 + axis_tw)
            painter.rotate(-90)
            painter.drawText(
                0, 0, axis_tw, aw,
                QtCore.Qt.AlignRight, '%i' % int(maxv))
            painter.restore()

            painter.save()
            painter.translate(ax + 2, wh / 2 - margin)
            painter.rotate(-90)
            painter.drawText(
                0, 0, axis_tw, aw,
                QtCore.Qt.AlignLeft, '%i' % int((maxv + minv) / 2))
            painter.restore()

            painter.save()
            painter.translate(ax + 2, wh / 2 + margin * 3 + axis_tw)
            painter.rotate(-90)
            painter.drawText(
                0, 0, axis_tw, aw,
                QtCore.Qt.AlignRight, label)
            painter.restore()

            denom = float(maxv - minv)
            ys = (
                (1. - (vs - minv) / denom)
                * (wh - margin * 2) + margin)
            painter.drawPolyline(
                QtGui.QPolygonF([
                    QtCore.QPointF(x, y) for (x, y)
                    in zip(xs[-len(ys):], ys)]))

        painter.end()


def main():
    app = QApplication(sys.argv)
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
