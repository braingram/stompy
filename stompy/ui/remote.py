#!/usr/bin/env python

import sys
import traceback

import numpy

from . import nogl
if nogl.has_qt5:
    from PyQt5 import QtCore, QtGui
    from . import base5 as base
    from PyQt5.QtWidgets import (
        QGestureEvent, QPinchGesture,
        QWidget, QApplication, QMainWindow,
        QAction, QInputDialog)
else:
    from PyQt4 import QtCore, QtGui
    from . import base as base
    from PyQt4.QtGui import (
        QGestureEvent, QPinchGesture,
        QWidget, QApplication, QMainWindow,
        QAction, QInputDialog)


from .. import calibration
from .. import consts
from .. import controllers
from .. import log
from .. import remote


class Tab(object):
    def __init__(self, ui, controller):
        self.ui = ui
        self.controller = controller
        if self.controller is not None:
            self._last_leg_index = None
            self.controller.on('', 'set_leg', self.set_leg_index)
            self.set_leg_index(
                self.controller.get('leg_index'))

    def set_leg_index(self, index):
        self._last_leg_index = index

    def start_showing(self):
        pass

    def stop_showing(self):
        pass


class PIDTab(Tab):
    n_points = 1000

    def __init__(self, ui, controller):
        self.chart = ui.pidLineChart
        self.chart.addSeries('Setpoint')
        self.chart.addSeries('Output')
        self.chart.addSeries('Error')

        super(PIDTab, self).__init__(ui, controller)
        self.joint_config = {}

        self.ui.pidJointCombo.currentIndexChanged.connect(
            self.change_joint)
        self.ui.pidCommitButton.clicked.connect(
            self.commit_values)

        self.change_joint()

    def set_leg_index(self, index):
        if self.controller is None:
            return
        if self._last_leg_index is not None:
            self.controller.remove_on(
                'legs[%s]' % self._last_leg_index,
                'pid', self.on_pid)
        super(PIDTab, self).set_leg_index(index)  # update index
        if index is not None:
            self.controller.on(
                'legs[%s]' % index,
                'pid', self.on_pid)

    def add_pid_values(self, output, setpoint, error):
        self.chart.appendData('Setpoint', setpoint)
        self.chart.appendData('Output', output)
        self.chart.appendData('Error', error)
        self.chart.update()

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
        txt = str(self.ui.pidJointCombo.currentText()).lower()
        self.joint_config = self.controller.call(
            'leg.pid_joint_config', txt)

        # set ui elements by joint_config
        if 'pid' in self.joint_config:
            if 'p' in self.joint_config['pid']:
                self.ui.pidPSpin.setValue(self.joint_config['pid']['p'])
            if 'i' in self.joint_config['pid']:
                self.ui.pidISpin.setValue(self.joint_config['pid']['i'])
            if 'd' in self.joint_config['pid']:
                self.ui.pidDSpin.setValue(self.joint_config['pid']['d'])
            if 'min' in self.joint_config['pid']:
                self.ui.pidMinSpin.setValue(self.joint_config['pid']['min'])
            if 'max' in self.joint_config['pid']:
                self.ui.pidMaxSpin.setValue(self.joint_config['pid']['max'])
        if 'pwm' in self.joint_config:
            if 'extend_min' in self.joint_config['pwm']:
                self.ui.extendMinSpin.setValue(
                    self.joint_config['pwm']['extend_min'])
            if 'extend_max' in self.joint_config['pwm']:
                self.ui.extendMaxSpin.setValue(
                    self.joint_config['pwm']['extend_max'])
            if 'retract_min' in self.joint_config['pwm']:
                self.ui.retractMinSpin.setValue(
                    self.joint_config['pwm']['retract_min'])
            if 'retract_max' in self.joint_config['pwm']:
                self.ui.retractMaxSpin.setValue(
                    self.joint_config['pwm']['retract_max'])
        if 'following_error_threshold' in self.joint_config:
            self.ui.pidErrorThresholdSpin.setValue(
                self.joint_config['following_error_threshold'])
        if 'adc' in self.joint_config:
            if 'min' in self.joint_config['adc']:
                self.ui.adcLimitMinSpin.setValue(
                    self.joint_config['adc']['min'])
            if 'max' in self.joint_config['adc']:
                self.ui.adcLimitMaxSpin.setValue(
                    self.joint_config['adc']['max'])
        if 'dither' in self.joint_config:
            if 'time' in self.joint_config['dither']:
                self.ui.ditherTimeSpin.setValue(
                    self.joint_config['dither']['time'])
            if 'amp' in self.joint_config['dither']:
                self.ui.ditherAmpSpin.setValue(
                    self.joint_config['dither']['amp'])

    def commit_values(self):
        # compare ui to joint config
        txt = str(self.ui.pidJointCombo.currentText()).lower()

        if txt not in consts.JOINT_INDEX_BY_NAME:
            return {}
        index = consts.JOINT_INDEX_BY_NAME[txt]

        settings = []
        # compare self.joint_config to values make settings
        if 'pid' in self.joint_config:
            j = self.joint_config['pid']
            v = {
                'p': self.ui.pidPSpin.value(),
                'i': self.ui.pidISpin.value(),
                'd': self.ui.pidDSpin.value(),
                'min': self.ui.pidMinSpin.value(),
                'max': self.ui.pidMaxSpin.value(),
            }

            if (
                    v['p'] != j['p'] or
                    v['i'] != j['i'] or
                    v['d'] != j['d'] or
                    v['min'] != j['min'] or
                    v['max'] != j['max']):
                settings.append((
                    'pid_config',
                    (index, v['p'], v['i'], v['d'], v['min'], v['max'])))

        if 'following_error_threshold' in self.joint_config:
            v = self.ui.pidErrorThresholdSpin.value()
            j = self.joint_config['following_error_threshold']
            if (v != j):
                settings.append((
                    'following_error_threshold', (index, float(v))))

        if 'pwm' in self.joint_config:
            j = self.joint_config['pwm']
            v = {
                'extend_min': self.ui.extendMinSpin.value(),
                'extend_max': self.ui.extendMaxSpin.value(),
                'retract_min': self.ui.retractMinSpin.value(),
                'retract_max': self.ui.retractMaxSpin.value(),
            }
            if (
                    v['extend_min'] != j['extend_min'] or
                    v['extend_max'] != j['extend_max'] or
                    v['retract_min'] != j['retract_min'] or
                    v['retract_max'] != j['retract_max']):
                settings.append((
                    'pwm_limits',
                    (index,
                    int(v['extend_min']),
                    int(v['extend_max']),
                    int(v['retract_min']),
                    int(v['retract_max']))))

        if 'adc' in self.joint_config:
            j = self.joint_config['adc']
            v = {
                'min': self.ui.adcLimitMinSpin.value(),
                'max': self.ui.adcLimitMaxSpin.value(),
            }
            if (v['min'] != j['min'] or v['max'] != j['max']):
                settings.append((
                    'adc_limits',
                    (index, v['min'], v['max'])))

        if 'dither' in self.joint_config:
            j = self.joint_config['dither']
            v = {
                'time': self.ui.ditherTimeSpin.value(),
                'amp': self.ui.ditherAmpSpin.value(),
            }
            if (v['time'] != j['time'] or v['amp'] != j['amp']):
                settings.append((
                    'dither',
                    (int(v['time']), int(v['amp']))))

        self.controller.call('leg.configure', settings)
        self.read_joint_config()

    def clear_pid_values(self):
        self.chart.clearData()
        self.chart.update()

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
            'azimuth': numpy.pi,
            'elevation': numpy.pi / 2,
            'offset': (-200, 0),
            'scalar': 3.,
        },
        'top': {
            'azimuth': 0,
            'elevation': 0,
            'offset': (-200, 0),
            'scalar': 3.,
        },
    }

    def __init__(self, ui, controller):
        self.display = ui.legDisplay
        super(LegTab, self).__init__(ui, controller)
        self.set_view('side')

    def set_leg_index(self, index):
        if self.controller is None:
            return
        if self._last_leg_index is not None:
            lo = 'legs[%i]' % self._last_leg_index
            self.controller.remove_on(lo, 'angles', self.on_angles)
            self.controller.remove_on(lo, 'xyz', self.on_xyz)
            self.controller.remove_on(lo, 'adc', self.on_adc)
            self.controller.remove_on(
                'res.feet[%i]' % self._last_leg_index,
                'restriction', self.on_restriction)
        super(LegTab, self).set_leg_index(index)  # update index
        self.display.leg.number = index
        self.display.update()
        if index is not None:
            lo = 'legs[%i]' % index
            self.controller.on(lo, 'angles', self.on_angles)
            self.controller.on(lo, 'xyz', self.on_xyz)
            self.controller.on(lo, 'adc', self.on_adc)
            self.controller.on(
                'res.feet[%i]' % index,
                'restriction', self.on_restriction)

    def set_view(self, view):
        if not isinstance(view, dict):
            view = self.views[view]
        for k in view:
            setattr(self.display.projection, k, view[k])
        self.display.update()

    def plot_leg(self, hip, thigh, knee):
        self.display.leg.set_angles(hip, thigh, knee)
        self.display.update()
        return

    def update_timer(self):
        self.plot_leg(self.angles[0], self.angles[1], self.angles[2])
        for i in xrange(3):
            self.angles[i] += self.deltas[i]
            if (
                    self.angles[i] > self.limits[i][0] or
                    self.angles[i] < self.limits[i][1]):
                self.deltas[i] *= -1

    def on_angles(self, angles):
        # TODO what to do when v is False?
        self.plot_leg(angles['hip'], angles['thigh'], angles['knee'])
        self.ui.legLLineEdit.setText('%0.2f' % angles['calf'])

    def on_xyz(self, xyz):
        self.ui.legXLineEdit.setText('%0.2f' % xyz['x'])
        self.ui.legYLineEdit.setText('%0.2f' % xyz['y'])
        self.ui.legZLineEdit.setText('%0.2f' % xyz['z'])

    def on_restriction(self, r):
        self.ui.legRLineEdit.setText('%0.2f' % r['r'])

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
            'azimuth': numpy.pi,
            'elevation': numpy.pi / 2.,
            'offset': (0, 0),
            'scalar': 1.,
        },
        'top': {
            'azimuth': numpy.pi,
            'elevation': 0,
            'offset': (0, 0),
            'scalar': 1.,
        },
    }

    def __init__(self, ui, controller):
        self.display = ui.bodyDisplay
        super(BodyTab, self).__init__(ui, controller)
        self.heightLabel = ui.heightLabel

        # attach to all legs
        self.controller.on('', 'height', self.on_height)
        self.controller.on('', 'mode', self.on_mode)
        for leg_number in self.controller.call('legs.keys'):
            self.display.add_leg(leg_number)
            lo = 'legs[%i]' % leg_number
            self.controller.on(
                lo, 'angles', lambda a, i=leg_number: self.on_angles(a, i))
            self.controller.on(
                lo, 'xyz', lambda a, i=leg_number: self.on_xyz(a, i))
            self.controller.on(
                lo, 'restriction',
                lambda a, i=leg_number: self.on_restriction(a, i))
            self.controller.on(
                'res.feet[%i]' % leg_number,
                'state', lambda a, i=leg_number: self.on_res_state(a, i))
        #self.show_top_view()
        self.set_view('top')

    def set_leg_index(self, index):
        if self.controller is None:
            return
        self.display.selected_leg = index
        super(BodyTab, self).set_leg_index(index)

    def set_view(self, view):
        if not isinstance(view, dict):
            view = self.views[view]
        for k in view:
            setattr(self.display.projection, k, view[k])
        self.display.update()

    def plot_leg(self, leg_number, hip, thigh, knee, calf):
        self.display.legs[leg_number].set_angles(hip, thigh, knee, calf)
        self.display.update()
        return

    def on_angles(self, angles, leg_number):
        self.plot_leg(
            leg_number, angles['hip'], angles['thigh'], angles['knee'],
            angles['calf'])

    def on_xyz(self, xyz, leg_number):
        pass

    def on_restriction(self, res, leg_number):
        self.display.legs[leg_number].restriction = res
        self.display.update()
        return

    def on_height(self, height):
        self.heightLabel.setText("Height: %0.2f" % height)

    def on_mode(self, mode):
        if mode == 'body_restriction':
            self._update_support_legs()
        else:
            self.display.support_legs = []

    def on_res_state(self, state, leg_number):
        self._update_support_legs()

    def _update_support_legs(self):
        # draw polygon between supported legs
        lns = sorted(self.controller.call('res.feet.keys'))
        support_legs = []
        for ln in lns:
            if (
                    self.controller.get('res.feet[%i].state' % ln)
                    in ('stance', 'wait')):
                support_legs.append(ln)
        if not len(support_legs):
            return
        self.display.support_legs = support_legs
        self.display.update()
        # TODO calculate pitch and roll


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
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = base.Ui_MainWindow()
    ui.setupUi(MainWindow)
    # setup menus
    ui._calibrationMenu_actions = []
    a = QAction("Save", ui.calibrationMenu)
    a.triggered.connect(lambda a: calibration.save_calibrations())
    ui._calibrationMenu_actions.append(a)
    ui.calibrationMenu.addAction(a)
    if controller is not None:
        a = QAction("Zero calf", ui.calibrationMenu)
        a.triggered.connect(
            lambda a: controller.call('leg.compute_calf_zero'))
        ui._calibrationMenu_actions.append(a)
        ui.calibrationMenu.addAction(a)

        ui._legsMenu_actions = []
        for leg in controller.call('legs.keys'):
            a = QAction(
                consts.LEG_NAME_BY_NUMBER[leg], ui.legsMenu)
            a.triggered.connect(
                lambda a, i=leg: controller.call('set_leg', i))
            ui._legsMenu_actions.append(a)
            ui.legsMenu.addAction(a)
        ui._modesMenu_actions = []
        for mode in controller.get('modes'):
            a = QAction(mode, ui.modesMenu)
            a.triggered.connect(
                lambda a, m=mode: controller.call('set_mode', m))
            ui._modesMenu_actions.append(a)
            ui.modesMenu.addAction(a)
        ui.modesMenu.setTitle("Mode: %s" % controller.get('mode'))
        ui.legsMenu.setTitle(
            "Leg: %s" % consts.LEG_NAME_BY_NUMBER[controller.get('leg_index')])
        controller.on(
            '', 'mode', lambda m: ui.modesMenu.setTitle("Mode: %s" % m))
        controller.on('', 'set_leg', lambda m: ui.legsMenu.setTitle(
            "Leg: %s" % consts.LEG_NAME_BY_NUMBER[m]))
        controller.on('', 'estop', lambda v: (
            ui.estopLabel.setText(
                "Estop: %s" % consts.ESTOP_BY_NUMBER[v]),
            ui.estopLabel.setStyleSheet((
                "background-color: green;" if v == consts.ESTOP_OFF else
                "background-color: none;"))
        ))
    tm = TabManager(ui.tabs)
    tm.add_tab('PID', PIDTab(ui, controller))
    tm.add_tab('Leg', LegTab(ui, controller))
    tm.add_tab('Body', BodyTab(ui, controller))
    tm.show_current()

    if 'imu' in controller.call('bodies.keys'):
        controller.on(
            'bodies["imu"]',
            'feed_pressure', lambda v: ui.pressureLabel.setText("PSI: %i" % v))
        controller.on(
            'bodies["imu"]',
            'engine_rpm', lambda v: ui.rpmLabel.setText("RPM: %0.0f" % v))
        controller.on(
            'bodies["imu"]',
            'feed_oil_temp',
            lambda v: ui.oilTempLabel.setText("Temp: %0.2f" % v))
        controller.on(
            'bodies["imu"]',
            'heading',
            lambda r, p, y: ui.imuLabel.setText(
                "IMU: %0.2f %0.2f %0.2f" % (r, p, y)))

    # param changes
    ui._configurationMenu_actions = []
    ui._configurationMenu_submenus = {}
    for name in sorted(controller.call('param.list_params')):
        # make submenus
        menu = ui.configurationMenu
        if '.' in name:
            # split submenus from name (leaving out last key)
            sn = name.split('.')
            subname = sn[-1]
            sn = sn[:-1]
            sd = ui._configurationMenu_submenus
            n = ""
            for ssn in sn:
                if len(n):
                    n = '%s.%s' % (n, ssn)
                else:
                    n = ssn
                if n not in sd:
                    #print("adding menu %s to %s" % (ssn, menu))
                    ui._configurationMenu_submenus[n] = menu.addMenu(ssn)
                menu = ui._configurationMenu_submenus[n]
            menu = ui._configurationMenu_submenus['.'.join(sn)]
        else:
            subname = name

        # add item to menu
        # when clicked show InputDialog (or have check mark)
        value = controller.get('param["%s"]' % name)
        if isinstance(value, bool):
            a = QAction(
                subname, menu, checkable=True, checked=value)
            a.triggered.connect(
                (
                    lambda value, n=name:
                    controller.call('param.set_param', n, value)))
            controller.on(
                'param', name, lambda nv, action=a: action.setChecked(nv))
        else:
            a = QAction('[%s] %s' % (value, subname), menu)

            def prompt_for_value(value, n=name):
                cv = controller.get('param["%s"]' % name)
                if isinstance(cv, float):
                    f = QInputDialog.getDouble
                else:
                    f = QInputDialog.getInt
                kwargs = controller.call('param.get_meta', n, {})
                nv, ok = f(MainWindow, n, n, cv, **kwargs)
                if ok:
                    controller.set('param["%s"]' % n, nv)

            if isinstance(value, float):
                fmt = '[%0.2f] %s'
            elif isinstance(value, int):
                fmt = '[%i] %s'
            else:
                fmt = '[%s] %s'

            def set_text(nv, action=a, n=subname, fmt=fmt):
                action.setText(fmt % (nv, n))

            # have title include value?
            controller.on(
                'param', name, set_text)
            #controller.param.on(
            #    name, lambda nv, action=a, n=name: action.setText(
            #        '[%s]%s' % (nv, n)))
            a.triggered.connect(prompt_for_value)
        ui._configurationMenu_actions.append(a)
        menu.addAction(a)

    MainWindow.show()
    #MainWindow.showFullScreen()
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
        timer.start(10)
    return {
        'app': app, 'ui': ui, 'window': MainWindow, 'tab_manager': tm,
        'timer': timer}


def run_ui(ui):
    sys.exit(ui['app'].exec_())


def start(remote_ui=False):
    if remote_ui:
        c = remote.client.RPCClient()
    else:
        c = remote.agent.RPCAgent(controllers.multileg.build())
        c.update = c.obj.update
    run_ui(load_ui(c))


if __name__ == "__main__":
    ui = load_ui()
    run_ui(ui)