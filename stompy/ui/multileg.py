#!/usr/bin/env python

import sys
import traceback

import numpy
#from PyQt4 import QtCore, QtGui

import pyqtgraph
import pyqtgraph.opengl
from pyqtgraph.Qt import QtCore, QtGui

pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')


from .. import consts
from . import base
from .. import kinematics
from .. import log


class Tab(object):
    def __init__(self, ui, controller):
        self.ui = ui
        self.controller = controller
        if self.controller is not None:
            self._last_leg_index = None
            self.controller.on('set_leg', self.set_leg_index)
            self.set_leg_index(self.controller.leg_index)

    def set_leg_index(self, index):
        self._last_leg_index = index

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

    def set_leg_index(self, index):
        if self.controller is None:
            return
        if self._last_leg_index is not None:
            self.controller.legs[self._last_leg_index].remove_on(
                'pid', self.on_pid)
        super(PIDTab, self).set_leg_index(index)  # update index
        if index is not None:
            self.controller.leg.on(
                'pid', self.on_pid)

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

    def on_pid(self, pid):
        txt = str(self.ui.pidJointCombo.currentText()).lower()
        o, s, e = pid['output'], pid['set_point'], pid['error']
        if txt not in o:
            return
        self.add_pid_values(o[txt], s[txt], e[txt])

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
        if (
                self.controller.leg is None or
                not hasattr(self.controller.leg, 'mgr')):
            return self.joint_config
        # get all values for this joint
        # P, I, D, min, max
        r = self.controller.leg.mgr.blocking_trigger('pid_config', index)
        self.joint_config['pid'] = {
            'p': r[1].value,
            'i': r[2].value,
            'd': r[3].value,
            'min': r[4].value,
            'max': r[5].value,
        }

        # following error threshold
        r = self.controller.leg.mgr.blocking_trigger(
            'following_error_threshold', index)
        self.joint_config['following_error_threshold'] = r[0].value

        # pwm: extend/retract min/max
        r = self.controller.leg.mgr.blocking_trigger('pwm_limits', index)
        self.joint_config['pwm'] = {
            'extend_min': r[1].value,
            'extend_max': r[2].value,
            'retract_min': r[3].value,
            'retract_max': r[4].value,
        }

        # adc limits
        r = self.controller.leg.mgr.blocking_trigger('adc_limits', index)
        self.joint_config['adc'] = {'min': r[1].value, 'max': r[2].value}

        # dither
        r = self.controller.leg.mgr.blocking_trigger('dither', index)
        self.joint_config['dither'] = {'time': r[1].value, 'amp': r[2].value}

        # seed time
        r = self.controller.leg.mgr.blocking_trigger('pid_seed_time')
        self.joint_config['seed_time'] = {
            'time': r[0].value, 'future': r[1].value}

        # set ui elements by joint_config
        self.ui.pidPSpin.setValue(self.joint_config['pid']['p'])
        self.ui.pidISpin.setValue(self.joint_config['pid']['i'])
        self.ui.pidDSpin.setValue(self.joint_config['pid']['d'])
        self.ui.pidMinSpin.setValue(self.joint_config['pid']['min'])
        self.ui.pidMaxSpin.setValue(self.joint_config['pid']['max'])
        self.ui.extendMinSpin.setValue(self.joint_config['pwm']['extend_min'])
        self.ui.extendMaxSpin.setValue(self.joint_config['pwm']['extend_max'])
        self.ui.pidErrorThresholdSpin.setValue(
            self.joint_config['following_error_threshold'])
        self.joint_config['following_error_threshold'] = r[0].value
        self.ui.retractMinSpin.setValue(
            self.joint_config['pwm']['retract_min'])
        self.ui.retractMaxSpin.setValue(
            self.joint_config['pwm']['retract_max'])
        self.ui.adcLimitMinSpin.setValue(self.joint_config['adc']['min'])
        self.ui.adcLimitMaxSpin.setValue(self.joint_config['adc']['max'])
        self.ui.ditherTimeSpin.setValue(self.joint_config['dither']['time'])
        self.ui.ditherAmpSpin.setValue(self.joint_config['dither']['amp'])
        self.ui.seedSpin.setValue(self.joint_config['seed_time']['time'])
        self.ui.seedFutureSpin.setValue(
            self.joint_config['seed_time']['future'])

    def commit_values(self):
        if (
                self.controller.leg is None or
                not hasattr(self.controller.leg, 'mgr')):
            return
        # compare to joint config
        # set ui elements by joint_config
        values = {
            'pid': {}, 'pwm': {}, 'adc': {}, 'dither': {},
            'seed_time': {}}
        values['pid']['p'] = self.ui.pidPSpin.value()
        values['pid']['i'] = self.ui.pidISpin.value()
        values['pid']['d'] = self.ui.pidDSpin.value()
        values['pid']['min'] = self.ui.pidMinSpin.value()
        values['pid']['max'] = self.ui.pidMaxSpin.value()
        values['following_error_threshold'] = \
            self.ui.pidErrorThresholdSpin.value()
        values['pwm']['extend_min'] = self.ui.extendMinSpin.value()
        values['pwm']['extend_max'] = self.ui.extendMaxSpin.value()
        values['pwm']['retract_min'] = self.ui.retractMinSpin.value()
        values['pwm']['retract_max'] = self.ui.retractMaxSpin.value()
        values['adc']['min'] = self.ui.adcLimitMinSpin.value()
        values['adc']['max'] = self.ui.adcLimitMaxSpin.value()
        values['dither']['time'] = self.ui.ditherTimeSpin.value()
        values['dither']['amp'] = self.ui.ditherAmpSpin.value()
        values['seed_time']['time'] = self.ui.seedSpin.value()
        values['seed_time']['future'] = self.ui.seedFutureSpin.value()

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
            # print("pid_config", args)
            log.info({'pid_config': args})
            self.controller.leg.mgr.trigger('pid_config', *args)

        v = values['following_error_threshold']
        j = self.joint_config['following_error_threshold']
        if (v != j):
            self.ui.pidErrorThresholdSpin.setValue(
                self.joint_config['following_error_threshold'])
            args = (index, v)
            log.info({'following_error_threshold': args})
            self.controller.leg.mgr.trigger(
                'following_error_threshold', *args)

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
            # print("pwm_limits:", args)
            log.info({'pwm_limits': args})
            self.controller.leg.mgr.trigger('pwm_limits', *args)
        v = values['adc']
        j = self.joint_config['adc']
        if (v['min'] != j['min'] or v['max'] != j['max']):
            args = (index, values['adc']['min'], values['adc']['max'])
            # print("adc_limits:", args)
            log.info({'adc_limits': args})
            self.controller.leg.mgr.trigger('adc_limits', *args)
        v = values['dither']
        j = self.joint_config['dither']
        if (v['time'] != j['time'] or v['amp'] != j['amp']):
            args = (
                index, int(values['dither']['time']),
                int(values['dither']['amp']))
            # print("dither:", args)
            log.info({'dither': args})
            self.controller.leg.mgr.trigger('dither', *args)
        v = values['seed_time']
        j = self.joint_config['seed_time']
        if (v['time'] != j['time'] or v['future'] != j['future']):
            args = (int(v['time']), int(v['future']))
            log.info({'pid_seed_time': args})
            self.controller.leg.mgr.trigger('pid_seed_time', *args)
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
            'azimuth': -90,
            'elevation': 0,
            'distance': 13000,
            'fov': 1,
        },
        'top': {
            'center': QtGui.QVector3D(60, 0, 0),
            'azimuth': -90,
            'elevation': 90,
            'distance': 12000,
            'fov': 1,
        },
    }

    def __init__(self, ui, controller):
        super(LegTab, self).__init__(ui, controller)
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
        pts = list(kinematics.leg.angles_to_points(0., 0., 0.))
        self.hip_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], pts[0]]), color=[1., 0., 0., 1.],
            width=5, antialias=True)
        self.thigh_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([pts[0], pts[1]]), color=[0., 1., 0., 1.],
            width=5, antialias=True)
        self.knee_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([pts[1], pts[2]]), color=[0., 0., 1., 1.],
            width=5, antialias=True)
        self.limit_links = []
        for z in [pts[2][2] - 6, pts[2][2], pts[2][2] + 6]:
            lpts = kinematics.leg.limits_at_z_3d(
                z, self._last_leg_index)
            if lpts is None:
                lpts = [[0, 0, 0], [1, 1, 1]]
            self.limit_links.append(pyqtgraph.opengl.GLLinePlotItem(
                pos=numpy.array(lpts), color=[0., 1., 1., 0.5],
                width=1, antialias=True))
            self.gl_widget.addItem(self.limit_links[-1])
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

    def set_leg_index(self, index):
        if self.controller is None:
            return
        if self._last_leg_index is not None:
            self.controller.legs[self._last_leg_index].remove_on(
                'angles', self.on_angles)
            self.controller.legs[self._last_leg_index].remove_on(
                'xyz', self.on_xyz)
            self.controller.legs[self._last_leg_index].remove_on(
                'adc', self.on_adc)
        super(LegTab, self).set_leg_index(index)  # update index
        if index is not None:
            self.controller.leg.on(
                'angles', self.on_angles)
            self.controller.leg.on(
                'xyz', self.on_xyz)
            self.controller.leg.on(
                'adc', self.on_adc)

    def on_gl_menu(self, point):
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
        pts = list(kinematics.leg.angles_to_points(hip, thigh, knee))
        self.hip_link.setData(pos=numpy.array([[0, 0, 0], pts[0]]))
        self.thigh_link.setData(pos=numpy.array([pts[0], pts[1]]))
        self.knee_link.setData(pos=numpy.array([pts[1], pts[2]]))
        lpts = kinematics.leg.limits_at_z_3d(
            pts[2][2], self._last_leg_index)
        for (i, z) in enumerate([pts[2][2] - 6, pts[2][2], pts[2][2] + 6]):
            lpts = kinematics.leg.limits_at_z_3d(
                z, self._last_leg_index)
            if lpts is not None:
                self.limit_links[i].setData(pos=numpy.array(lpts))

    def update_timer(self):
        self.plot_leg(self.angles[0], self.angles[1], self.angles[2])
        for i in xrange(3):
            self.angles[i] += self.deltas[i]
            if (
                    self.angles[i] > self.limits[i][0] or
                    self.angles[i] < self.limits[i][1]):
                self.deltas[i] *= -1

    def on_angles(self, angles):
        # TODO h, t, k readouts
        # TODO what to do when v is False?
        self.plot_leg(angles['hip'], angles['thigh'], angles['knee'])
        self.ui.legLLineEdit.setText('%0.2f' % angles['calf'])

    def on_xyz(self, xyz):
        self.ui.legXLineEdit.setText('%0.2f' % xyz['x'])
        self.ui.legYLineEdit.setText('%0.2f' % xyz['y'])
        self.ui.legZLineEdit.setText('%0.2f' % xyz['z'])
        # TODO restriction?

    def on_adc(self, adc):
        self.ui.hipADCProgress.setValue(adc['hip'])
        self.ui.thighADCProgress.setValue(adc['thigh'])
        self.ui.kneeADCProgress.setValue(adc['knee'])
        self.ui.calfADCProgress.setValue(adc['calf'])

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


class BodyTab(Tab):
    views = {
        'back': {
            'center': QtGui.QVector3D(0, 0, 0),
            'azimuth': -90,
            'elevation': 0,
            'distance': 30000,
            'fov': 1,
        },
        'top': {
            'center': QtGui.QVector3D(0, 0, 0),
            'azimuth': -90,
            'elevation': 90,
            'distance': 40000,
            'fov': 1,
        },
    }

    def __init__(self, ui, controller):
        super(BodyTab, self).__init__(ui, controller)
        self.gl_widget = ui.bodyGLWidget

        # add grids
        gs = 12
        gi = gs * 30
        self.grids = {}
        self.grids['x'] = pyqtgraph.opengl.GLGridItem()
        self.grids['x'].setSize(gi, gi, 1)
        self.grids['x'].setSpacing(gs, gs, 1)
        self.grids['x'].rotate(90, 0, 1, 0)
        #self.grids['x'].translate(gi / 2, 0, 0)
        self.gl_widget.addItem(self.grids['x'])
        self.grids['y'] = pyqtgraph.opengl.GLGridItem()
        self.grids['y'].setSize(gi, gi, 1)
        self.grids['y'].setSpacing(gs, gs, 1)
        self.grids['y'].rotate(90, 1, 0, 0)
        #self.grids['y'].translate(gi / 2, 0, 0)
        self.gl_widget.addItem(self.grids['y'])
        self.grids['z'] = pyqtgraph.opengl.GLGridItem()
        self.grids['z'].setSize(gi, gi, 1)
        self.grids['z'].setSpacing(gs, gs, 1)
        #self.grids['z'].translate(gi / 2, 0, 0)
        self.gl_widget.addItem(self.grids['z'])

        # add origin
        d = 12
        x_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], [d, 0, 0]]), color=[1., 0., 0., 1.],
            width=2, antialias=True)
        self.gl_widget.addItem(x_link)
        y_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], [0, d, 0]]), color=[0., 1., 0., 1.],
            width=2, antialias=True)
        self.gl_widget.addItem(y_link)
        z_link = pyqtgraph.opengl.GLLinePlotItem(
            pos=numpy.array([[0, 0, 0], [0, 0, d]]), color=[0., 0., 1., 1.],
            width=2, antialias=True)
        self.gl_widget.addItem(z_link)

        # add legs
        self.links = {}
        for leg_number in self.controller.legs:
            pts = numpy.array([
                [0., 0., 0.], ] + list(
                    kinematics.leg.angles_to_points(0., 0., 0.)))
            pts = kinematics.body.leg_to_body_array(
                leg_number, pts)
            hip_link = pyqtgraph.opengl.GLLinePlotItem(
                pos=numpy.array([pts[0], pts[1]]), color=[1., 0., 0., 1.],
                width=5, antialias=True)
            thigh_link = pyqtgraph.opengl.GLLinePlotItem(
                pos=numpy.array([pts[1], pts[2]]), color=[0., 1., 0., 1.],
                width=5, antialias=True)
            knee_link = pyqtgraph.opengl.GLLinePlotItem(
                pos=numpy.array([pts[2], pts[3]]), color=[0., 0., 1., 1.],
                width=5, antialias=True)
            calf_link = pyqtgraph.opengl.GLScatterPlotItem(
                pos=pts[3], color=[1., 0.5, 0., 1.], size=1., pxMode=True)
            limit_links = []
            for z in [pts[3][2] - 6, pts[3][2], pts[3][2] + 6]:
                lpts = kinematics.leg.limits_at_z_3d(
                    z, leg_number)
                if lpts is None:
                    lpts = [[0, 0, 0], [1, 1, 1]]
                limit_links.append(pyqtgraph.opengl.GLLinePlotItem(
                    pos=numpy.array(lpts), color=[0., 1., 1., 0.5],
                    width=1, antialias=True))
                self.gl_widget.addItem(limit_links[-1])
            self.gl_widget.addItem(hip_link)
            self.gl_widget.addItem(thigh_link)
            self.gl_widget.addItem(knee_link)
            self.gl_widget.addItem(calf_link)
            self.links[leg_number] = {
                'hip': hip_link, 'thigh': thigh_link, 'knee': knee_link,
                'calf': calf_link, 'limit': limit_links}

        # add context menu
        self.gl_widget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.gl_widget.customContextMenuRequested.connect(self.on_gl_menu)
        self.gl_menu = QtGui.QMenu(self.gl_widget)
        sva = QtGui.QAction('Back view', self.gl_widget)
        sva.triggered.connect(self.show_back_view)
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

        # attach to all legs
        #self.controller.legs[i]
        for leg_number in self.controller.legs:
            self.controller.legs[leg_number].on(
                'angles', lambda a, i=leg_number: self.on_angles(a, i))
            self.controller.legs[leg_number].on(
                'xyz', lambda a, i=leg_number: self.on_xyz(a, i))
        self.show_top_view()

    def on_gl_menu(self, point):
        self.gl_menu.exec_(self.gl_widget.mapToGlobal(point))

    def show_grids(self):
        for k in self.grids:
            self.grids[k].setVisible(True)

    def hide_grids(self):
        for k in self.grids:
            self.grids[k].setVisible(False)

    def show_back_view(self):
        self.gl_widget.opts.update(self.views['back'])
        self.gl_widget.update()

    def show_top_view(self):
        self.gl_widget.opts.update(self.views['top'])
        self.gl_widget.update()

    def plot_leg(self, leg_number, hip, thigh, knee, calf):

        pts = [[0., 0., 0.], ] + list(
            kinematics.leg.angles_to_points(hip, thigh, knee))
        pts = kinematics.body.leg_to_body_array(leg_number, pts)
        self.links[leg_number]['hip'].setData(
            pos=numpy.array([pts[0], pts[1]]))
        self.links[leg_number]['thigh'].setData(
            pos=numpy.array([pts[1], pts[2]]))
        self.links[leg_number]['knee'].setData(
            pos=numpy.array([pts[2], pts[3]]))
        self.links[leg_number]['calf'].setData(
            pos=pts[3], size=calf/50.)
        for (i, z) in enumerate([pts[3][2] - 6, pts[3][2], pts[3][2] + 6]):
            lpts = kinematics.leg.limits_at_z_3d(z, leg_number)
            if lpts is not None:
                lpts = kinematics.body.leg_to_body_array(
                    leg_number, numpy.array(lpts))
                self.links[leg_number]['limit'][i].setData(
                    pos=numpy.array(lpts))

    def on_angles(self, angles, leg_number):
        self.plot_leg(
            leg_number, angles['hip'], angles['thigh'], angles['knee'],
            angles['calf'])

    def on_xyz(self, xyz, leg_number):
        # TODO restriction?
        pass


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
        # lookup index
        label = str(self.tab_widget.tabText(index))
        if self.current is not None:
            self.current.stop_showing()
            self.current = None
        if label in self.tabs:
            self.tabs[label].start_showing()
            self.current = self.tabs[label]


def load_ui(controller=None):
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = base.Ui_MainWindow()
    ui.setupUi(MainWindow)
    # setup menu
    if controller is not None:
        ui._legsMenu_actions = []
        for leg in controller.legs:
            a = QtGui.QAction(
                consts.LEG_NAME_BY_NUMBER[leg], ui.legsMenu)
            a.triggered.connect(lambda a, i=leg: controller.set_leg(i))
            ui._legsMenu_actions.append(a)
            ui.legsMenu.addAction(a)
        ui._modesMenu_actions = []
        for mode in controller.modes:
            a = QtGui.QAction(mode, ui.modesMenu)
            a.triggered.connect(lambda a, m=mode: controller.set_mode(m))
            ui._modesMenu_actions.append(a)
            ui.modesMenu.addAction(a)
        ui.modeLabel.setText("Mode: %s" % controller.mode)
        ui.legLabel.setText(
            "Leg: %s" % consts.LEG_NAME_BY_NUMBER[controller.leg_index])
        controller.on('mode', lambda m: ui.modeLabel.setText("Mode: %s" % m))
        controller.on('set_leg', lambda m: ui.legLabel.setText(
            "Leg: %s" % consts.LEG_NAME_BY_NUMBER[m]))
    tm = TabManager(ui.tabs)
    tm.add_tab('PID', PIDTab(ui, controller))
    tm.add_tab('Leg', LegTab(ui, controller))
    tm.add_tab('Body', BodyTab(ui, controller))
    tm.show_current()

    def set_values(item):
        if item.columnCount() == 2:
            parent = item.parent()
            if parent is not None:
                parent = str(parent.text(0))
            attr, value = str(item.text(0)), str(item.text(1))
            if parent is not None:
                attr = '.'.join((parent, attr))
            ts = attr.split('.')
            obj = controller
            assert ts[0] == 'controller'
            ts = ts[1:]
            while len(ts) > 1:
                obj = getattr(obj, ts.pop(0))
                if obj is None:
                    break
            if obj is not None:
                attr = ts[0]
                if isinstance(obj, dict):
                    old_value = obj[attr]
                else:
                    old_value = getattr(obj, attr)
                item.setText(1, str(old_value))
        for i in range(item.childCount()):
            set_values(item.child(i))

    # update tree widget to show values from python
    set_values(ui.configTree.invisibleRootItem())
    ui.configTree.resizeColumnToContents(0)
    # listen for configTree changes

    def item_changed(item):
        # TODO recurse through parents
        parent = item.parent()
        if parent is not None:
            parent = str(parent.text(0))
        attr, value = str(item.text(0)), str(item.text(1))
        print(parent, attr, value)
        if parent is not None:
            attr = '.'.join((parent, attr))
        # get old value
        ts = attr.split('.')
        obj = controller
        assert ts[0] == 'controller'
        ts = ts[1:]
        while len(ts) > 1:
            obj = getattr(obj, ts.pop(0))
        attr = ts[0]
        if isinstance(obj, dict):
            old_value = obj[attr]
        else:
            old_value = getattr(obj, attr)
        print("Old value: %s" % old_value)
        try:
            value = type(old_value)(value)
        except Exception as e:
            print("Error converting value %s: %s [%s]" % (attr, value, e))
            item.setText(1, str(old_value))
            return
        try:
            if isinstance(obj, dict):
                obj[attr] = value
            else:
                setattr(obj, attr, value)
        except Exception as e:
            print("Error setting config %s = %s [%s]" % (attr, value, e))
            item.setText(1, str(old_value))
            return

    ui.configTree.itemChanged.connect(item_changed)
    MainWindow.show()
    timer = None
    if controller is not None:
        timer = QtCore.QTimer()

        def update():
            try:
                controller.update()
            except Exception as e:
                ex_type, ex, tb = sys.exc_info()
                print("controller update error: %s" % e)
                traceback.print_tb(tb)
                # TODO stop updates on error?
                #timer.stop()
        #timer.timeout.connect(controller.update)
        timer.timeout.connect(update)
        timer.start(1)
    return {
        'app': app, 'ui': ui, 'window': MainWindow, 'tab_manager': tm,
        'timer': timer}


def run_ui(ui):
    sys.exit(ui['app'].exec_())


if __name__ == "__main__":
    ui = load_ui()
    run_ui(ui)
