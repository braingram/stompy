#!/usr/bin/env python

import sys

import numpy
#from PyQt4 import QtCore, QtGui

import pyqtgraph
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')


import base


def setup_pid_plot(pw):
    pw.show()
    pw.setWindowTitle("Blah")
    p1 = pw.plotItem
    p1.getAxis('left').setLabel("Output")
    p1.getAxis('left').setPen('g')

    p2 = pyqtgraph.ViewBox()
    p1.showAxis('right')
    p1.scene().addItem(p2)
    p1.getAxis('right').linkToView(p2)
    p2.setXLink(p1)
    p1.getAxis('right').setLabel("SetPoint")
    p1.getAxis('right').setPen('b')

    p3 = pyqtgraph.ViewBox()
    ax3 = pyqtgraph.AxisItem('right')
    p3._ax3 = ax3
    p1.layout.addItem(ax3, 2, 3)
    p1.scene().addItem(p3)
    ax3.linkToView(p3)
    p3.setXLink(p1)
    ax3.setZValue(-10000)
    ax3.setLabel("Error")
    ax3.setPen('r')

    def updateViews(a=0, p1=p1, p2=p2, p3=p3):
        print(a, p1, p2, p3)
        ## view has resized; update auxiliary views to match
        p2.setGeometry(p1.vb.sceneBoundingRect())
        p3.setGeometry(p1.vb.sceneBoundingRect())

        ## need to re-update linked axes since this was called
        ## incorrectly while views had different shapes.
        ## (probably this should be handled in ViewBox.resizeEvent)
        p2.linkedViewChanged(p1.vb, p2.XAxis)
        p3.linkedViewChanged(p1.vb, p3.XAxis)

    updateViews()
    p1.vb.sigResized.connect(updateViews)

    v = list(numpy.linspace(-8192, 8192, 500))
    d1 = p1.plot(v, pen='g')
    d1._data = v
    v = list(numpy.linspace(65535, 0, 500))
    d2 = pyqtgraph.PlotCurveItem(v, pen='b')
    d2._data = v
    p2.addItem(d2)
    v = list(numpy.random.randint(-65535, 65535, 500))
    d3 = pyqtgraph.PlotCurveItem(v, pen='r')
    d3._data = v
    p3.addItem(d3)
    print(d1, d2, d3)

    def update(d1=d1, d2=d2, d3=d3):
        #print(d1, d2, d3)
        for d in (d1, d2, d3):
            d._data.append(d._data[0])
            d._data = d._data[1:]
            d.setData(d._data)
        # force redraw?
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0)
    d1._timer = timer


def test():
    pyqtgraph.mkQApp()
    pw = pyqtgraph.PlotWidget()
    setup_pid_plot(pw)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test()
        sys.exit(1)
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = base.Ui_MainWindow()
    ui.setupUi(MainWindow)
    setup_pid_plot(ui.pidPlotWidget)
    MainWindow.show()
    sys.exit(app.exec_())
