#!/usr/bin/env python
"""
data could be as simple as a list of lists

each column would need it's own:
- scalar 
- axis (multiple on right?)
- color
"""

import sys

import numpy
from PyQt4 import (QtGui, QtCore)

import stompy


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


class LineChart(QtGui.QWidget):
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

            ys = (
                (1. - (vs - minv) / float(maxv - minv))
                * (wh - margin * 2) + margin)
            painter.drawPolyline(
                QtGui.QPolygonF([
                    QtCore.QPointF(x, y) for (x, y)
                    in zip(xs[-len(ys):], ys)]))

        painter.end()


def main():
    app = QtGui.QApplication(sys.argv)
    #w = QtGui.QWidget()
    w = LineChart()
    w.resize(300, 300)
    w.move(300, 300)
    w.setWindowTitle('Simple')
    w.show()
    w.addSeries('setpoint')
    w.addSeries('output')
    w.addSeries('error')
    w.appendData('setpoint', 0)
    w.appendData('setpoint', 100)
    w.appendData('output', 0)
    w.appendData('output', -100)
    w.appendData('error', 0)
    w.appendData('error', 500)

    def tick():
        for l in ('setpoint', 'output', 'error'):
            v0, v1 = w.data[l].data[-2:]
            dv = v1 - v0
            if abs(v1 + dv) > 65535:
                dv *= -1
            w.appendData(l, v1 + dv)
        #w.appendData('setpoint', numpy.random.randint(-65535, 65535))
        #w.appendData('output', numpy.random.randint(-65535, 65535))
        #w.appendData('error', numpy.random.randint(-65535, 65535))

        w.update()

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
