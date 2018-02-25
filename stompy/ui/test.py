#!/usr/bin/env python

import sys

import numpy
#from PyQt4 import QtCore, QtGui

import pyqtgraph
import pyqtgraph.opengl
from pyqtgraph.Qt import QtCore, QtGui

pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')


import base


HIP_LENGTH = 11
THIGH_LENGTH = 54
KNEE_LENGTH = 72
THIGH_REST_ANGLE = 1.4663392033212757
KNEE_REST_ANGLE = -1.4485440219526171


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
        #print(a, p1, p2, p3)
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
    timer.start(50)
    d1._timer = timer


def test():
    pyqtgraph.mkQApp()
    pw = pyqtgraph.PlotWidget()
    setup_pid_plot(pw)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


class Tab(object):
    def __init__(self, ui):
        self.ui = ui

    def start_showing(self):
        pass

    def stop_showing(self):
        pass


class PIDTab(Tab):
    def __init__(self, ui):
        super(PIDTab, self).__init__(ui)
        self.plot = ui.pidPlotWidget
        self.output_line = self.plot.plotItem
        self.output_line.getAxis('left').setLabel("Output")
        self.output_line.getAxis('left').setPen("g")

        self.setpoint_line = pyqtgraph.ViewBox()
        self.output_line.showAxis('right')
        self.output_line.scene().addItem(self.setpoint_line)
        self.output_line.getAxis('right').linkToView(self.setpoint_line)
        self.setpoint_line.setXLink(self.output_line)
        self.output_line.getAxis('right').setLabel("SetPoint")
        self.output_line.getAxis('right').setPen("b")

        self.error_line = pyqtgraph.ViewBox()
        ax = pyqtgraph.AxisItem('right')
        self.error_line._ax = ax
        self.output_line.layout.addItem(ax, 2, 3)
        self.output_line.scene().addItem(self.error_line)
        ax.linkToView(self.error_line)
        self.error_line.setXLink(self.output_line)
        ax.setZValue(-10000)
        ax.setLabel("Error")
        ax.setPen("r")

        self.update_views()
        self.output_line.vb.sigResized.connect(self.update_views)

        v = []
        self.output_data = self.output_line.plot(v, pen='g')
        self.output_data._data = v
        v = []
        self.setpoint_data = pyqtgraph.PlotCurveItem(v, pen='b')
        self.setpoint_line.addItem(self.setpoint_data)
        self.setpoint_data._data = v
        v = []
        self.error_data = pyqtgraph.PlotCurveItem(v, pen='r')
        self.error_line.addItem(self.error_data)
        self.error_data._data = v

    def update_views(self):
        self.setpoint_line.setGeometry(self.output_line.vb.sceneBoundingRect())
        self.error_line.setGeometry(self.output_line.vb.sceneBoundingRect())

        self.setpoint_line.linkedViewChanged(
            self.output_line.vb, self.setpoint_line.XAxis)
        self.error_line.linkedViewChanged(
            self.output_line.vb, self.error_line.XAxis)

    def add_pid_values(self, output, setpoint, error):
        for (v, d) in zip(
                (output, setpoint, error),
                (self.output_data, self.setpoint_data, self.error_data)):
            d._data.append(v)
            while len(d._data) > 200:
                d._data.pop(0)
            d.setData(d._data)

    def clear_pid_values(self):
        for d in (self.output_data, self.setpoint_data, self.error_data):
            d._data = []
            d.setData(d._data)

    def timer_update(self):
        rd = list(numpy.random.random(3))
        self.add_pid_values(*rd)

    def start_showing(self):
        self.clear_pid_values()
        print("starting timer")
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(50)

    def stop_showing(self):
        self.timer.stop()


class LegTab(Tab):
    views = {
        'side': {
            'center': QtGui.QVector3D(60, 0, 0),
            'azimuth': 90,
            'elevation': 0,
            'distance': 9000,
            'fov': 1,
        },
        'top': {
            'center': QtGui.QVector3D(60, 0, 0),
            'azimuth': 90,
            'elevation': 90,
            'distance': 9000,
            'fov': 1,
        },
    }

    def __init__(self, ui):
        super(LegTab, self).__init__(ui)
        self.gl_widget = ui.legGLWidget
        # add grids
        # add leg links
        pts = list(generate_leg_points(0., 0., 0.))
        self.hip_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], pts[0]]), color=[1., 0., 0., 1.],
            width=5, antialias=True)
        self.thigh_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([pts[0], pts[1]]), color=[0., 1., 0., 1.],
            width=5, antialias=True)
        self.knee_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([pts[1], pts[2]]), color=[0., 0., 1., 1.],
            width=5, antialias=True)
        self.gl_widget.addItem(self.hip_link)
        self.gl_widget.addItem(self.thigh_link)
        self.gl_widget.addItem(self.knee_link)

    def plot_leg(self, hip, thigh, knee):
        pts = list(generate_leg_points(hip, thigh, knee))
        self.hip_link.setData(pos=numpy.array([[0, 0, 0], pts[0]]))
        self.thigh_link.setData(pos=numpy.array([pts[0], pts[1]]))
        self.knee_link.setData(pos=numpy.array([pts[1], pts[2]]))

    def update_timer(self):
        self.plot_leg(self.angles[0], self.angles[1], self.angles[2])
        for i in xrange(3):
            self.angles[i] += self.deltas[i]
            if (
                    self.angles[i] > self.limits[i][0] or
                    self.angles[i] < self.limits[i][1]):
                self.deltas[i] *= -1

    def start_showing(self):
        self.angles = [0., 0., 0.]
        self.deltas = [0.01, 0.01, -0.02]
        self.limits = [[0.6, -0.6], [1.1, 0.], [0., -1.1]]
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_timer)
        self.timer.start(50)

    def stop_showing(self):
        self.timer.stop()


class TabManager(object):
    def __init__(self, tab_widget):
        self.tab_widget = tab_widget
        self.tab_widget.currentChanged.connect(self.tab_changed)
        self.tabs = {}
        # TODO start showing current
        self.current = None

    def add_tab(self, name, tab):
        self.tabs[name] = tab

    def tab_changed(self, index):
        print("Tab changed to index: %s" % index)
        # lookup index
        label = str(self.tab_widget.tabText(index))
        print("  label: %s" % label)
        if self.current is not None:
            print("stop showing: %s" % self.current)
            self.current.stop_showing()
            self.current = None
        if label in self.tabs:
            print("start showing: %s" % self.tabs[label])
            self.tabs[label].start_showing()
            self.current = self.tabs[label]


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test()
        sys.exit(1)
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = base.Ui_MainWindow()
    ui.setupUi(MainWindow)
    tm = TabManager(ui.tabs)
    tm.add_tab('PID', PIDTab(ui))
    tm.add_tab('Leg', LegTab(ui))
    MainWindow.show()
    sys.exit(app.exec_())
