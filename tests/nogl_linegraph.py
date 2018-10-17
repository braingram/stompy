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
        QtGui.QPen(QtCore.Qt.red, 1),  # hip link
        QtGui.QPen(QtCore.Qt.green, 1),  # thigh link
        QtGui.QPen(QtCore.Qt.blue, 1),  # calf link
    ]

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
        # TODO give space for axes at left
        xs = numpy.arange(ChartData.max_n) * ww / float(ChartData.max_n)
        for (series_i, label) in enumerate(self.data):
            # TODO draw axis
            # plot data
            vs = numpy.array(self.data[label].get_data())
            minv, maxv = numpy.min(vs), numpy.max(vs)
            # scale to y (0 is top)
            ys = (1. - (vs - minv) / (maxv - minv)) * wh
            painter.setPen(self.data[label]._pen)
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

    def tick():
        w.appendData('setpoint', numpy.random.random())
        w.appendData('output', numpy.random.random())
        w.appendData('error', numpy.random.random())

        w.update()

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
