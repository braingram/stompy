#!/usr/bin/env python

import sys

import numpy
#from PyQt4 import QtCore, QtGui

import pyqtgraph
import pyqtgraph.opengl
from pyqtgraph.Qt import QtCore, QtGui

pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')


from . import base
from .. import log


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


class Tab(object):
    def __init__(self, ui, controller):
        self.ui = ui
        self.controller = controller

    def start_showing(self):
        pass

    def stop_showing(self):
        pass


class PIDTab(Tab):
    n_points = 1000

    def __init__(self, ui, controller):
        super(PIDTab, self).__init__(ui, controller)
        # TODO make ui map
        self.joint_config = {}
        if self.controller is not None:
            self.controller.conn.mgr.on('report_pid', self.on_report_pid)

        self.ui.pidJointCombo.currentIndexChanged.connect(
            self.change_joint)
        self.ui.pidCommitButton.clicked.connect(
            self.commit_values)

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
        self.change_joint()

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
            while len(d._data) > self.n_points:
                d._data.pop(0)
            d.setData(d._data)

    def on_report_pid(self, ho, to, ko, hs, ts, ks, he, te, ke):
        txt = str(self.ui.pidJointCombo.currentText())
        if txt == 'Hip':
            o, s, e = ho.value, hs.value, he.value
        elif txt == 'Thigh':
            o, s, e = to.value, ts.value, te.value
        elif txt == 'Knee':
            o, s, e = ko.value, ks.value, ke.value
        else:
            return
        self.add_pid_values(o, s, e)

    def change_joint(self):
        self.clear_pid_values()
        self.read_joint_config()

    def read_joint_config(self):
        # get current joint
        txt = str(self.ui.pidJointCombo.currentText())
        try:
            index = ['Hip', 'Thigh', 'Knee'].index(txt)
        except ValueError:
            return
        self.joint_config = {}
        # get all values for this joint
        # P, I, D, min, max
        r = self.controller.conn.mgr.blocking_trigger('pid_config', index)
        self.joint_config['pid'] = {
            'p': r[1].value,
            'i': r[2].value,
            'd': r[3].value,
            'min': r[4].value,
            'max': r[5].value,
        }

        # pwm: extend/retract min/max
        r = self.controller.conn.mgr.blocking_trigger('pwm_limits', index)
        self.joint_config['pwm'] = {
            'extend_min': r[1].value,
            'extend_max': r[2].value,
            'retract_min': r[3].value,
            'retract_max': r[4].value,
        }

        # adc limits
        r = self.controller.conn.mgr.blocking_trigger('adc_limits', index)
        self.joint_config['adc'] = {'min': r[1].value, 'max': r[2].value}

        # dither
        r = self.controller.conn.mgr.blocking_trigger('dither', index)
        self.joint_config['dither'] = {'time': r[1].value, 'amp': r[2].value}

        # set ui elements by joint_config
        self.ui.pidPSpin.setValue(self.joint_config['pid']['p'])
        self.ui.pidISpin.setValue(self.joint_config['pid']['i'])
        self.ui.pidDSpin.setValue(self.joint_config['pid']['d'])
        self.ui.pidMinSpin.setValue(self.joint_config['pid']['min'])
        self.ui.pidMaxSpin.setValue(self.joint_config['pid']['max'])
        self.ui.extendMinSpin.setValue(self.joint_config['pwm']['extend_min'])
        self.ui.extendMaxSpin.setValue(self.joint_config['pwm']['extend_max'])
        self.ui.retractMinSpin.setValue(
            self.joint_config['pwm']['retract_min'])
        self.ui.retractMaxSpin.setValue(
            self.joint_config['pwm']['retract_max'])
        self.ui.adcLimitMinSpin.setValue(self.joint_config['adc']['min'])
        self.ui.adcLimitMaxSpin.setValue(self.joint_config['adc']['max'])
        self.ui.ditherTimeSpin.setValue(self.joint_config['dither']['time'])
        self.ui.ditherAmpSpin.setValue(self.joint_config['dither']['amp'])

    def commit_values(self):
        # compare to joint config
        # set ui elements by joint_config
        values = {'pid': {}, 'pwm': {}, 'adc': {}, 'dither': {}}
        values['pid']['p'] = self.ui.pidPSpin.value()
        values['pid']['i'] = self.ui.pidISpin.value()
        values['pid']['d'] = self.ui.pidDSpin.value()
        values['pid']['min'] = self.ui.pidMinSpin.value()
        values['pid']['max'] = self.ui.pidMaxSpin.value()
        values['pwm']['extend_min'] = self.ui.extendMinSpin.value()
        values['pwm']['extend_max'] = self.ui.extendMaxSpin.value()
        values['pwm']['retract_min'] = self.ui.retractMinSpin.value()
        values['pwm']['retract_max'] = self.ui.retractMaxSpin.value()
        values['adc']['min'] = self.ui.adcLimitMinSpin.value()
        values['adc']['max'] = self.ui.adcLimitMaxSpin.value()
        values['dither']['time'] = self.ui.ditherTimeSpin.value()
        values['dither']['amp'] = self.ui.ditherAmpSpin.value()

        txt = str(self.ui.pidJointCombo.currentText())
        try:
            index = ['Hip', 'Thigh', 'Knee'].index(txt)
        except ValueError:
            return

        v = values['pid']
        j = self.joint_config['pid']
        if (
                v['p'] != j['p'] or v['i'] != j['i'] or v['d'] != j['i']):
            args = (
                index,
                values['pid']['p'], values['pid']['i'], values['pid']['d'],
                values['pid']['min'], values['pid']['max'])
            print("pid_config", args)
            log.info({'pid_config': args})
            self.controller.conn.mgr.trigger('pid_config', *args)
        v = values['pwm']
        j = self.joint_config['pwm']
        if (
                v['extend_min'] != j['extend_min'] or
                v['extend_max'] != j['extend_max'] or
                v['retract_min'] != j['retract_min'] or
                v['retract_max'] != j['retract_max']):
            args = (
                index,
                int(values['pwm']['extend_min']),
                int(values['pwm']['extend_max']),
                int(values['pwm']['retract_min']),
                int(values['pwm']['retract_max']))
            print("pwm_limits:", args)
            log.info({'pwm_limits': args})
            self.controller.conn.mgr.trigger('pwm_limits', *args)
        v = values['adc']
        j = self.joint_config['adc']
        if (v['min'] != j['min'] or v['max'] != j['max']):
            args = (index, values['adc']['min'], values['adc']['max'])
            print("adc_limits:", args)
            log.info({'adc_limits': args})
            self.controller.conn.mgr.trigger('adc_limits', *args)
        v = values['dither']
        j = self.joint_config['dither']
        if (v['time'] != j['time'] or v['amp'] != j['amp']):
            args = (
                index, int(values['dither']['time']),
                int(values['dither']['amp']))
            print("dither:", args)
            log.info({'dither': args})
            self.controller.conn.mgr.trigger('dither', *args)
        self.read_joint_config()

    def clear_pid_values(self):
        for d in (self.output_data, self.setpoint_data, self.error_data):
            d._data = []
            d.setData(d._data)

    def timer_update(self):
        rd = list(numpy.random.random(3))
        self.add_pid_values(*rd)

    def start_showing(self):
        self.clear_pid_values()
        if self.controller is None:
            print("starting timer")
            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self.timer_update)
            self.timer.start(50)

    def stop_showing(self):
        if self.controller is None:
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

    def __init__(self, ui, controller):
        super(LegTab, self).__init__(ui, controller)
        if self.controller is not None:
            self.controller.conn.mgr.on('report_angles', self.on_report_angles)
            self.controller.conn.mgr.on('report_xyz', self.on_report_xyz)
            self.controller.conn.mgr.on('report_adc', self.on_report_adc)
            #self.controller.conn.mgr.on('pwm_value', self.on_pwm_value)

        self.gl_widget = ui.legGLWidget
        # add grids
        gi = 120
        self.grids = {}
        self.grids['x'] = pyqtgraph.opengl.GLGridItem()
        self.grids['x'].setSize(gi, gi, 1)
        self.grids['x'].setSpacing(6, 6, 1)
        self.grids['x'].rotate(90, 0, 1, 0)
        #self.grids['x'].translate(-25, 0, 0)
        self.gl_widget.addItem(self.grids['x'])
        self.grids['y'] = pyqtgraph.opengl.GLGridItem()
        self.grids['y'].setSize(gi, gi, 1)
        self.grids['y'].setSpacing(6, 6, 1)
        self.grids['y'].rotate(90, 1, 0, 0)
        self.grids['y'].translate(gi / 2, 0, 0)
        self.gl_widget.addItem(self.grids['y'])
        self.grids['z'] = pyqtgraph.opengl.GLGridItem()
        self.grids['z'].setSize(gi, gi, 1)
        self.grids['z'].setSpacing(6, 6, 1)
        self.grids['z'].translate(gi / 2, 0, 0)
        self.gl_widget.addItem(self.grids['z'])

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

        # make context menu
        self.gl_widget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.gl_widget.customContextMenuRequested.connect(self.on_gl_menu)
        self.gl_menu = QtGui.QMenu(self.gl_widget)
        sva = QtGui.QAction('Side view', self.gl_widget)
        sva.triggered.connect(self.show_side_view)
        self.gl_menu.addAction(sva)
        tva = QtGui.QAction('Top view', self.gl_widget)
        tva.triggered.connect(self.show_top_view)
        self.gl_menu.addAction(tva)
        self.gl_menu.addSeparator()
        sga = QtGui.QAction('Show grids', self.gl_widget)
        sga.triggered.connect(self.show_grids)
        self.gl_menu.addAction(sga)
        hga = QtGui.QAction('Hide grids', self.gl_widget)
        hga.triggered.connect(self.hide_grids)
        self.gl_menu.addAction(hga)
        #self.gl_menu.triggered[QtGui.QAction].connect(self.on_gl_menu_action)
        self.show_side_view()

    def on_gl_menu(self, point):
        print(point)
        self.gl_menu.exec_(self.gl_widget.mapToGlobal(point))

    #def on_gl_menu_action(self, action):
    #    print(action)
    #    print(action.text())

    def show_grids(self):
        for k in self.grids:
            self.grids[k].setVisible(True)

    def hide_grids(self):
        for k in self.grids:
            self.grids[k].setVisible(False)

    def show_side_view(self):
        self.gl_widget.opts.update(self.views['side'])
        self.gl_widget.update()

    def show_top_view(self):
        self.gl_widget.opts.update(self.views['top'])
        self.gl_widget.update()

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

    def on_report_angles(self, h, t, k, c, v):
        # TODO h, t, k readouts
        # TODO what to do when v is False?
        self.plot_leg(h.value, t.value, k.value)
        self.ui.legLLineEdit.setText('%0.2f' % c.value)

    def on_report_xyz(self, x, y, z):
        self.ui.legXLineEdit.setText('%0.2f' % x.value)
        self.ui.legYLineEdit.setText('%0.2f' % y.value)
        self.ui.legZLineEdit.setText('%0.2f' % z.value)
        # TODO restriction?

    def on_report_adc(self, h, t, k, c):
        self.ui.hipADCProgress.setValue(h.value)
        self.ui.thighADCProgress.setValue(t.value)
        self.ui.kneeADCProgress.setValue(k.value)
        self.ui.calfADCProgress.setValue(c.value)

    def start_showing(self):
        if self.controller is None:
            self.angles = [0., 0., 0.]
            self.deltas = [0.01, 0.01, -0.02]
            self.limits = [[0.6, -0.6], [1.1, 0.], [0., -1.1]]
            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self.update_timer)
            self.timer.start(50)

    def stop_showing(self):
        if self.controller is None:
            self.timer.stop()


class TabManager(object):
    def __init__(self, tab_widget):
        self.tab_widget = tab_widget
        self.tab_widget.currentChanged.connect(self.tab_changed)
        self.tabs = {}
        self.current = None

    def show_current(self):
        i = self.tab_widget.currentIndex()
        if i == -1:
            return
        self.tab_changed(i)

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


def load_ui(controller=None):
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = base.Ui_MainWindow()
    ui.setupUi(MainWindow)
    tm = TabManager(ui.tabs)
    tm.add_tab('PID', PIDTab(ui, controller))
    tm.add_tab('Leg', LegTab(ui, controller))
    tm.show_current()
    MainWindow.show()
    timer = None
    if controller is not None:
        timer = QtCore.QTimer()
        timer.timeout.connect(controller.update)
        timer.start(1)
    return {
        'app': app, 'ui': ui, 'window': MainWindow, 'tab_manager': tm,
        'timer': timer}


def run_ui(ui):
    sys.exit(ui['app'].exec_())


if __name__ == "__main__":
    ui = load_ui()
    run_ui(ui)
