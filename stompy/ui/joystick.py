#!/usr/bin/env python

import time

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import (
    QGestureEvent, QPinchGesture,
    QSwipeGesture,
    QWidget, QApplication, QMainWindow,
    QAction, QInputDialog)

from .. import consts


class FakeJoystickUI(QWidget):
    def __init__(self, joy):
        super(FakeJoystickUI, self).__init__()
        self.target = (0., 0.)  # -1 to 1
        self.joy = joy
        self.deadman = 0

        # setup timer to send deadman every consts.HEARTBEAT_PERIOD
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.send_deadman)
        self.timer.start(int(consts.HEARTBEAT_PERIOD * 1000.))
        self.joy.on('buttons', self.on_joy_buttons)

    def on_joy_buttons(self, buttons):
        if buttons.get('deadman', 1) == 0 and self.deadman:
            print("setting self.deadman to 0")
            self.deadman = 0

    def closeEvent(self, event):
        self.timer.stop()
        del self.timer

    def send_deadman(self):
        #print("sending deadman:", time.time())
        self.joy._report_button('deadman', self.deadman)

    def send_target(self):
        x = int((self.target[0] + 1) * (255 / 2.))
        y = int((self.target[1] + 1) * (255 / 2.))
        self.joy._report_axis('x', x)
        self.joy._report_axis('y', y)
   
    def mousePressEvent(self, event):
        # if event.button() == QtCore.Qt.RightButton:
        #     latching = True
        # else:
        #     latching = False
        pt = event.pos()
        x, y = pt.x(), pt.y()
        w, h = (self.width(), self.height())
        hw, hh = (w / 2, h / 2)
        self.target = (
            (x - hw) / float(hw),
            -(y - hh) / float(hh),
        )
        self.send_target()
        self.update()

    def keyPressEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_Shift:
            #print("set deadman")
            self.deadman = 1
            self.send_deadman()
            self.send_target()

    def keyReleaseEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_Shift:
            #print("release deadman")
            self.deadman = 0
            self.send_deadman()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)
        # draw line from center to target
        w, h = (self.width(), self.height())
        hw, hh = (w / 2, h / 2)
        dx = int(self.target[0] * hw)
        dy = int(self.target[1] * hh)
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.drawLine(hw, hh, hw + dx, hh - dy)
        painter.end()


def make_joystick_window(joy):
    ui = FakeJoystickUI(joy)
    ui.resize(400, 400)
    ui.show()
    return ui
