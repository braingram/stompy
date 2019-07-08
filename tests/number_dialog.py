#!/usr/bin/env python

import os
import sys

from PyQt5 import QtCore, QtGui, uic
#from . import base5 as base
from PyQt5.QtWidgets import (
    QGestureEvent, QPinchGesture,
    QSwipeGesture, QPushButton, QDialog,
    QWidget, QApplication, QMainWindow,
    QAction, QInputDialog)


ui_filename = os.path.abspath(os.path.expanduser('../stompy/ui/dialog.ui'))
print(ui_filename)

var = {
    'value': 1.0,
    'min': -5.,
    'max': 5.,
    'step': 0.1,
    'decimals': 2,
}


def launch_dialog():
    print("starting value: %s" % var['value'])
    dialog = QDialog()
    ui = uic.loadUi(ui_filename, dialog)

    # setup value display
    ui.doubleSpinBox.setMinimum(var['min'])
    ui.doubleSpinBox.setMaximum(var['max'])
    ui.doubleSpinBox.setSingleStep(var['step'])
    ui.doubleSpinBox.setDecimals(var['decimals'])
    ui.doubleSpinBox.setValue(var['value'])
    ui.spin_text = ""

    def clear_text():
        ui.spin_text = ""
        ui.doubleSpinBox.clear()

    # connect to clear button
    ui.clearButton.clicked.connect(clear_text)

    def next_value(clicked, v):
        if v == '-':
            if len(ui.spin_text) and ui.spin_text == '-':
                ui.spin_text = ui.spin_text[1:]
            else:
                ui.spin_text = '-' + ui.spin_text
        else:
            ui.spin_text += str(v)
        #print(v, str(v), ui.spin_text)
        sv = ui.doubleSpinBox.valueFromText(ui.spin_text)
        #print(sv)
        ui.doubleSpinBox.setValue(sv)

    # connect up buttons
    for i in range(10):
        w = getattr(ui, 'pushButton_%i' % i)
        w.clicked.connect(lambda c, v=i: next_value(c, v))

    ui.pushButton_p.clicked.connect(lambda c: next_value(c, '.'))
    ui.pushButton_n.clicked.connect(lambda c: next_value(c, '-'))

    # TODO slider?

    if dialog.exec_():  # 0 if closed/canceled, 1 if OK
        # set new value
        var['value'] = type(var['value'])(ui.doubleSpinBox.value())
        print("New value: %s" % var['value'])


app = QApplication(sys.argv)
win = QMainWindow()
btn = QPushButton(win)
btn.setText("Hi")
btn.move(50, 50)
btn.clicked.connect(launch_dialog)
win.show()

sys.exit(app.exec_())
