# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'stompy.ui'
#
# Created: Tue Mar 20 19:20:14 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(756, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.estopCheck = QtGui.QCheckBox(self.centralwidget)
        self.estopCheck.setObjectName(_fromUtf8("estopCheck"))
        self.horizontalLayout_8.addWidget(self.estopCheck)
        self.legLabel = QtGui.QLabel(self.centralwidget)
        self.legLabel.setObjectName(_fromUtf8("legLabel"))
        self.horizontalLayout_8.addWidget(self.legLabel)
        self.modeLabel = QtGui.QLabel(self.centralwidget)
        self.modeLabel.setObjectName(_fromUtf8("modeLabel"))
        self.horizontalLayout_8.addWidget(self.modeLabel)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.tabs = QtGui.QTabWidget(self.centralwidget)
        self.tabs.setObjectName(_fromUtf8("tabs"))
        self.legTab = QtGui.QWidget()
        self.legTab.setObjectName(_fromUtf8("legTab"))
        self.verticalLayout_15 = QtGui.QVBoxLayout(self.legTab)
        self.verticalLayout_15.setObjectName(_fromUtf8("verticalLayout_15"))
        self.legGLWidget = GLViewWidget(self.legTab)
        self.legGLWidget.setMinimumSize(QtCore.QSize(0, 0))
        self.legGLWidget.setObjectName(_fromUtf8("legGLWidget"))
        self.verticalLayout_15.addWidget(self.legGLWidget)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label = QtGui.QLabel(self.legTab)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_3.addWidget(self.label)
        self.hipADCProgress = QtGui.QProgressBar(self.legTab)
        self.hipADCProgress.setMaximum(65535)
        self.hipADCProgress.setProperty("value", 6500)
        self.hipADCProgress.setObjectName(_fromUtf8("hipADCProgress"))
        self.horizontalLayout_3.addWidget(self.hipADCProgress)
        self.horizontalLayout_7.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_2 = QtGui.QLabel(self.legTab)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_4.addWidget(self.label_2)
        self.thighADCProgress = QtGui.QProgressBar(self.legTab)
        self.thighADCProgress.setMaximum(65535)
        self.thighADCProgress.setProperty("value", 6500)
        self.thighADCProgress.setObjectName(_fromUtf8("thighADCProgress"))
        self.horizontalLayout_4.addWidget(self.thighADCProgress)
        self.horizontalLayout_7.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.label_3 = QtGui.QLabel(self.legTab)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.horizontalLayout_5.addWidget(self.label_3)
        self.kneeADCProgress = QtGui.QProgressBar(self.legTab)
        self.kneeADCProgress.setMaximum(65535)
        self.kneeADCProgress.setProperty("value", 6500)
        self.kneeADCProgress.setObjectName(_fromUtf8("kneeADCProgress"))
        self.horizontalLayout_5.addWidget(self.kneeADCProgress)
        self.horizontalLayout_7.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.label_4 = QtGui.QLabel(self.legTab)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_6.addWidget(self.label_4)
        self.calfADCProgress = QtGui.QProgressBar(self.legTab)
        self.calfADCProgress.setMaximum(65535)
        self.calfADCProgress.setProperty("value", 6500)
        self.calfADCProgress.setObjectName(_fromUtf8("calfADCProgress"))
        self.horizontalLayout_6.addWidget(self.calfADCProgress)
        self.horizontalLayout_7.addLayout(self.horizontalLayout_6)
        self.verticalLayout_15.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_73 = QtGui.QHBoxLayout()
        self.horizontalLayout_73.setObjectName(_fromUtf8("horizontalLayout_73"))
        self.horizontalLayout_71 = QtGui.QHBoxLayout()
        self.horizontalLayout_71.setObjectName(_fromUtf8("horizontalLayout_71"))
        self.label_70 = QtGui.QLabel(self.legTab)
        self.label_70.setObjectName(_fromUtf8("label_70"))
        self.horizontalLayout_71.addWidget(self.label_70)
        self.legXLineEdit = QtGui.QLineEdit(self.legTab)
        self.legXLineEdit.setObjectName(_fromUtf8("legXLineEdit"))
        self.horizontalLayout_71.addWidget(self.legXLineEdit)
        self.horizontalLayout_73.addLayout(self.horizontalLayout_71)
        self.horizontalLayout_70 = QtGui.QHBoxLayout()
        self.horizontalLayout_70.setObjectName(_fromUtf8("horizontalLayout_70"))
        self.label_69 = QtGui.QLabel(self.legTab)
        self.label_69.setObjectName(_fromUtf8("label_69"))
        self.horizontalLayout_70.addWidget(self.label_69)
        self.legYLineEdit = QtGui.QLineEdit(self.legTab)
        self.legYLineEdit.setObjectName(_fromUtf8("legYLineEdit"))
        self.horizontalLayout_70.addWidget(self.legYLineEdit)
        self.horizontalLayout_73.addLayout(self.horizontalLayout_70)
        self.horizontalLayout_9 = QtGui.QHBoxLayout()
        self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
        self.label_5 = QtGui.QLabel(self.legTab)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_9.addWidget(self.label_5)
        self.legZLineEdit = QtGui.QLineEdit(self.legTab)
        self.legZLineEdit.setObjectName(_fromUtf8("legZLineEdit"))
        self.horizontalLayout_9.addWidget(self.legZLineEdit)
        self.horizontalLayout_73.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_72 = QtGui.QHBoxLayout()
        self.horizontalLayout_72.setObjectName(_fromUtf8("horizontalLayout_72"))
        self.label_71 = QtGui.QLabel(self.legTab)
        self.label_71.setObjectName(_fromUtf8("label_71"))
        self.horizontalLayout_72.addWidget(self.label_71)
        self.legLLineEdit = QtGui.QLineEdit(self.legTab)
        self.legLLineEdit.setObjectName(_fromUtf8("legLLineEdit"))
        self.horizontalLayout_72.addWidget(self.legLLineEdit)
        self.horizontalLayout_73.addLayout(self.horizontalLayout_72)
        self.horizontalLayout_69 = QtGui.QHBoxLayout()
        self.horizontalLayout_69.setObjectName(_fromUtf8("horizontalLayout_69"))
        self.label_68 = QtGui.QLabel(self.legTab)
        self.label_68.setObjectName(_fromUtf8("label_68"))
        self.horizontalLayout_69.addWidget(self.label_68)
        self.legRLineEdit = QtGui.QLineEdit(self.legTab)
        self.legRLineEdit.setObjectName(_fromUtf8("legRLineEdit"))
        self.horizontalLayout_69.addWidget(self.legRLineEdit)
        self.horizontalLayout_73.addLayout(self.horizontalLayout_69)
        self.verticalLayout_15.addLayout(self.horizontalLayout_73)
        self.tabs.addTab(self.legTab, _fromUtf8(""))
        self.PIDTab = QtGui.QWidget()
        self.PIDTab.setObjectName(_fromUtf8("PIDTab"))
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.PIDTab)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.pidPlotWidget = PlotWidget(self.PIDTab)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.pidPlotWidget.sizePolicy().hasHeightForWidth())
        self.pidPlotWidget.setSizePolicy(sizePolicy)
        self.pidPlotWidget.setObjectName(_fromUtf8("pidPlotWidget"))
        self.verticalLayout_6.addWidget(self.pidPlotWidget)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.label_29 = QtGui.QLabel(self.PIDTab)
        self.label_29.setObjectName(_fromUtf8("label_29"))
        self.verticalLayout_5.addWidget(self.label_29)
        self.horizontalLayout_24 = QtGui.QHBoxLayout()
        self.horizontalLayout_24.setObjectName(_fromUtf8("horizontalLayout_24"))
        self.label_30 = QtGui.QLabel(self.PIDTab)
        self.label_30.setObjectName(_fromUtf8("label_30"))
        self.horizontalLayout_24.addWidget(self.label_30)
        self.pidPSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.pidPSpin.setSingleStep(0.1)
        self.pidPSpin.setObjectName(_fromUtf8("pidPSpin"))
        self.horizontalLayout_24.addWidget(self.pidPSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_24)
        self.horizontalLayout_25 = QtGui.QHBoxLayout()
        self.horizontalLayout_25.setObjectName(_fromUtf8("horizontalLayout_25"))
        self.label_31 = QtGui.QLabel(self.PIDTab)
        self.label_31.setObjectName(_fromUtf8("label_31"))
        self.horizontalLayout_25.addWidget(self.label_31)
        self.pidISpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.pidISpin.setSingleStep(0.1)
        self.pidISpin.setObjectName(_fromUtf8("pidISpin"))
        self.horizontalLayout_25.addWidget(self.pidISpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_25)
        self.horizontalLayout_26 = QtGui.QHBoxLayout()
        self.horizontalLayout_26.setObjectName(_fromUtf8("horizontalLayout_26"))
        self.label_32 = QtGui.QLabel(self.PIDTab)
        self.label_32.setObjectName(_fromUtf8("label_32"))
        self.horizontalLayout_26.addWidget(self.label_32)
        self.pidDSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.pidDSpin.setSingleStep(0.1)
        self.pidDSpin.setObjectName(_fromUtf8("pidDSpin"))
        self.horizontalLayout_26.addWidget(self.pidDSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_26)
        self.horizontalLayout_75 = QtGui.QHBoxLayout()
        self.horizontalLayout_75.setObjectName(_fromUtf8("horizontalLayout_75"))
        self.label_73 = QtGui.QLabel(self.PIDTab)
        self.label_73.setObjectName(_fromUtf8("label_73"))
        self.horizontalLayout_75.addWidget(self.label_73)
        self.pidMinSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.pidMinSpin.setDecimals(2)
        self.pidMinSpin.setMinimum(-8192.0)
        self.pidMinSpin.setMaximum(8192.0)
        self.pidMinSpin.setSingleStep(1.0)
        self.pidMinSpin.setObjectName(_fromUtf8("pidMinSpin"))
        self.horizontalLayout_75.addWidget(self.pidMinSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_75)
        self.horizontalLayout_74 = QtGui.QHBoxLayout()
        self.horizontalLayout_74.setObjectName(_fromUtf8("horizontalLayout_74"))
        self.label_72 = QtGui.QLabel(self.PIDTab)
        self.label_72.setObjectName(_fromUtf8("label_72"))
        self.horizontalLayout_74.addWidget(self.label_72)
        self.pidMaxSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.pidMaxSpin.setMinimum(-8192.0)
        self.pidMaxSpin.setMaximum(8192.0)
        self.pidMaxSpin.setSingleStep(1.0)
        self.pidMaxSpin.setObjectName(_fromUtf8("pidMaxSpin"))
        self.horizontalLayout_74.addWidget(self.pidMaxSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_74)
        spacerItem = QtGui.QSpacerItem(20, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.line = QtGui.QFrame(self.PIDTab)
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.horizontalLayout_2.addWidget(self.line)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_25 = QtGui.QLabel(self.PIDTab)
        self.label_25.setObjectName(_fromUtf8("label_25"))
        self.verticalLayout_4.addWidget(self.label_25)
        self.horizontalLayout_21 = QtGui.QHBoxLayout()
        self.horizontalLayout_21.setObjectName(_fromUtf8("horizontalLayout_21"))
        self.label_26 = QtGui.QLabel(self.PIDTab)
        self.label_26.setObjectName(_fromUtf8("label_26"))
        self.horizontalLayout_21.addWidget(self.label_26)
        self.extendMinSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.extendMinSpin.setDecimals(0)
        self.extendMinSpin.setMaximum(8192.0)
        self.extendMinSpin.setObjectName(_fromUtf8("extendMinSpin"))
        self.horizontalLayout_21.addWidget(self.extendMinSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_21)
        self.horizontalLayout_22 = QtGui.QHBoxLayout()
        self.horizontalLayout_22.setObjectName(_fromUtf8("horizontalLayout_22"))
        self.label_27 = QtGui.QLabel(self.PIDTab)
        self.label_27.setObjectName(_fromUtf8("label_27"))
        self.horizontalLayout_22.addWidget(self.label_27)
        self.extendMaxSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.extendMaxSpin.setDecimals(0)
        self.extendMaxSpin.setMaximum(8192.0)
        self.extendMaxSpin.setObjectName(_fromUtf8("extendMaxSpin"))
        self.horizontalLayout_22.addWidget(self.extendMaxSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_22)
        self.horizontalLayout_23 = QtGui.QHBoxLayout()
        self.horizontalLayout_23.setObjectName(_fromUtf8("horizontalLayout_23"))
        self.label_28 = QtGui.QLabel(self.PIDTab)
        self.label_28.setObjectName(_fromUtf8("label_28"))
        self.horizontalLayout_23.addWidget(self.label_28)
        self.retractMinSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.retractMinSpin.setDecimals(0)
        self.retractMinSpin.setMaximum(8192.0)
        self.retractMinSpin.setObjectName(_fromUtf8("retractMinSpin"))
        self.horizontalLayout_23.addWidget(self.retractMinSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_23)
        self.horizontalLayout_30 = QtGui.QHBoxLayout()
        self.horizontalLayout_30.setObjectName(_fromUtf8("horizontalLayout_30"))
        self.label_33 = QtGui.QLabel(self.PIDTab)
        self.label_33.setObjectName(_fromUtf8("label_33"))
        self.horizontalLayout_30.addWidget(self.label_33)
        self.retractMaxSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.retractMaxSpin.setDecimals(0)
        self.retractMaxSpin.setMaximum(8192.0)
        self.retractMaxSpin.setObjectName(_fromUtf8("retractMaxSpin"))
        self.horizontalLayout_30.addWidget(self.retractMaxSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_30)
        spacerItem1 = QtGui.QSpacerItem(20, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem1)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.line_2 = QtGui.QFrame(self.PIDTab)
        self.line_2.setFrameShape(QtGui.QFrame.VLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.horizontalLayout_2.addWidget(self.line_2)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.label_12 = QtGui.QLabel(self.PIDTab)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.verticalLayout_2.addWidget(self.label_12)
        self.horizontalLayout_11 = QtGui.QHBoxLayout()
        self.horizontalLayout_11.setObjectName(_fromUtf8("horizontalLayout_11"))
        self.label_11 = QtGui.QLabel(self.PIDTab)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.horizontalLayout_11.addWidget(self.label_11)
        self.adcLimitMinSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.adcLimitMinSpin.setDecimals(0)
        self.adcLimitMinSpin.setMaximum(65535.0)
        self.adcLimitMinSpin.setObjectName(_fromUtf8("adcLimitMinSpin"))
        self.horizontalLayout_11.addWidget(self.adcLimitMinSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_10 = QtGui.QHBoxLayout()
        self.horizontalLayout_10.setObjectName(_fromUtf8("horizontalLayout_10"))
        self.label_10 = QtGui.QLabel(self.PIDTab)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.horizontalLayout_10.addWidget(self.label_10)
        self.adcLimitMaxSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.adcLimitMaxSpin.setDecimals(0)
        self.adcLimitMaxSpin.setMaximum(65535.0)
        self.adcLimitMaxSpin.setObjectName(_fromUtf8("adcLimitMaxSpin"))
        self.horizontalLayout_10.addWidget(self.adcLimitMaxSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_10)
        self.label_13 = QtGui.QLabel(self.PIDTab)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.verticalLayout_2.addWidget(self.label_13)
        self.horizontalLayout_12 = QtGui.QHBoxLayout()
        self.horizontalLayout_12.setObjectName(_fromUtf8("horizontalLayout_12"))
        self.label_14 = QtGui.QLabel(self.PIDTab)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.horizontalLayout_12.addWidget(self.label_14)
        self.ditherTimeSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.ditherTimeSpin.setDecimals(0)
        self.ditherTimeSpin.setMaximum(1000000.0)
        self.ditherTimeSpin.setObjectName(_fromUtf8("ditherTimeSpin"))
        self.horizontalLayout_12.addWidget(self.ditherTimeSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_13 = QtGui.QHBoxLayout()
        self.horizontalLayout_13.setObjectName(_fromUtf8("horizontalLayout_13"))
        self.label_15 = QtGui.QLabel(self.PIDTab)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.horizontalLayout_13.addWidget(self.label_15)
        self.ditherAmpSpin = QtGui.QDoubleSpinBox(self.PIDTab)
        self.ditherAmpSpin.setDecimals(0)
        self.ditherAmpSpin.setMaximum(8192.0)
        self.ditherAmpSpin.setObjectName(_fromUtf8("ditherAmpSpin"))
        self.horizontalLayout_13.addWidget(self.ditherAmpSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_13)
        spacerItem2 = QtGui.QSpacerItem(20, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem2)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.verticalLayout_6.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_29 = QtGui.QHBoxLayout()
        self.horizontalLayout_29.setObjectName(_fromUtf8("horizontalLayout_29"))
        self.pidCommitButton = QtGui.QPushButton(self.PIDTab)
        self.pidCommitButton.setObjectName(_fromUtf8("pidCommitButton"))
        self.horizontalLayout_29.addWidget(self.pidCommitButton)
        self.pidJointCombo = QtGui.QComboBox(self.PIDTab)
        self.pidJointCombo.setObjectName(_fromUtf8("pidJointCombo"))
        self.pidJointCombo.addItem(_fromUtf8(""))
        self.pidJointCombo.addItem(_fromUtf8(""))
        self.pidJointCombo.addItem(_fromUtf8(""))
        self.horizontalLayout_29.addWidget(self.pidJointCombo)
        self.verticalLayout_6.addLayout(self.horizontalLayout_29)
        self.tabs.addTab(self.PIDTab, _fromUtf8(""))
        self.bodyTab = QtGui.QWidget()
        self.bodyTab.setObjectName(_fromUtf8("bodyTab"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.bodyTab)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.bodyGLWidget = GLViewWidget(self.bodyTab)
        self.bodyGLWidget.setMinimumSize(QtCore.QSize(0, 0))
        self.bodyGLWidget.setObjectName(_fromUtf8("bodyGLWidget"))
        self.verticalLayout_3.addWidget(self.bodyGLWidget)
        self.tabs.addTab(self.bodyTab, _fromUtf8(""))
        self.walkTab = QtGui.QWidget()
        self.walkTab.setObjectName(_fromUtf8("walkTab"))
        self.tabs.addTab(self.walkTab, _fromUtf8(""))
        self.horizontalLayout.addWidget(self.tabs)
        self.verticalLayout.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 756, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.legsMenu = QtGui.QMenu(self.menubar)
        self.legsMenu.setObjectName(_fromUtf8("legsMenu"))
        self.modesMenu = QtGui.QMenu(self.menubar)
        self.modesMenu.setObjectName(_fromUtf8("modesMenu"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.legsMenu.menuAction())
        self.menubar.addAction(self.modesMenu.menuAction())

        self.retranslateUi(MainWindow)
        self.tabs.setCurrentIndex(3)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.estopCheck.setText(_translate("MainWindow", "CheckBox", None))
        self.legLabel.setText(_translate("MainWindow", "Leg: ?", None))
        self.modeLabel.setText(_translate("MainWindow", "Mode: ?", None))
        self.label.setText(_translate("MainWindow", "Hip", None))
        self.hipADCProgress.setFormat(_translate("MainWindow", "%v", None))
        self.label_2.setText(_translate("MainWindow", "Thigh", None))
        self.thighADCProgress.setFormat(_translate("MainWindow", "%v", None))
        self.label_3.setText(_translate("MainWindow", "Knee", None))
        self.kneeADCProgress.setFormat(_translate("MainWindow", "%v", None))
        self.label_4.setText(_translate("MainWindow", "Calf", None))
        self.calfADCProgress.setFormat(_translate("MainWindow", "%v", None))
        self.label_70.setText(_translate("MainWindow", "X", None))
        self.label_69.setText(_translate("MainWindow", "Y", None))
        self.label_5.setText(_translate("MainWindow", "Z", None))
        self.label_71.setText(_translate("MainWindow", "L", None))
        self.label_68.setText(_translate("MainWindow", "R", None))
        self.tabs.setTabText(self.tabs.indexOf(self.legTab), _translate("MainWindow", "Leg", None))
        self.label_29.setText(_translate("MainWindow", "PID", None))
        self.label_30.setText(_translate("MainWindow", "P", None))
        self.label_31.setText(_translate("MainWindow", "I", None))
        self.label_32.setText(_translate("MainWindow", "D", None))
        self.label_73.setText(_translate("MainWindow", "Min", None))
        self.label_72.setText(_translate("MainWindow", "Max", None))
        self.label_25.setText(_translate("MainWindow", "PWM", None))
        self.label_26.setText(_translate("MainWindow", "Extend min", None))
        self.label_27.setText(_translate("MainWindow", "Extend max", None))
        self.label_28.setText(_translate("MainWindow", "Retract min", None))
        self.label_33.setText(_translate("MainWindow", "Retract max", None))
        self.label_12.setText(_translate("MainWindow", "ADC limits", None))
        self.label_11.setText(_translate("MainWindow", "Min", None))
        self.label_10.setText(_translate("MainWindow", "Max", None))
        self.label_13.setText(_translate("MainWindow", "Dither", None))
        self.label_14.setText(_translate("MainWindow", "Time(us)", None))
        self.label_15.setText(_translate("MainWindow", "Amplitude", None))
        self.pidCommitButton.setText(_translate("MainWindow", "Commit", None))
        self.pidJointCombo.setItemText(0, _translate("MainWindow", "Hip", None))
        self.pidJointCombo.setItemText(1, _translate("MainWindow", "Thigh", None))
        self.pidJointCombo.setItemText(2, _translate("MainWindow", "Knee", None))
        self.tabs.setTabText(self.tabs.indexOf(self.PIDTab), _translate("MainWindow", "PID", None))
        self.tabs.setTabText(self.tabs.indexOf(self.bodyTab), _translate("MainWindow", "Body", None))
        self.tabs.setTabText(self.tabs.indexOf(self.walkTab), _translate("MainWindow", "Walk", None))
        self.legsMenu.setTitle(_translate("MainWindow", "Legs", None))
        self.modesMenu.setTitle(_translate("MainWindow", "Modes", None))

from pyqtgraph.opengl import GLViewWidget
from pyqtgraph import PlotWidget

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

