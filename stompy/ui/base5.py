# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'stompy.ui'
#
# Created: Sun Mar 17 14:06:38 2019
#      by: PyQt5 UI code generator 5.2.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 480)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.tabs = QtWidgets.QTabWidget(self.centralwidget)
        self.tabs.setTabPosition(QtWidgets.QTabWidget.East)
        self.tabs.setObjectName("tabs")
        self.legTab = QtWidgets.QWidget()
        self.legTab.setObjectName("legTab")
        self.legDisplay = LegDisplay(self.legTab)
        self.legDisplay.setGeometry(QtCore.QRect(10, 10, 400, 400))
        self.legDisplay.setMinimumSize(QtCore.QSize(0, 0))
        self.legDisplay.setObjectName("legDisplay")
        self.layoutWidget = QtWidgets.QWidget(self.legTab)
        self.layoutWidget.setGeometry(QtCore.QRect(580, 170, 126, 27))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setObjectName("label")
        self.horizontalLayout_3.addWidget(self.label)
        self.hipADCProgress = QtWidgets.QProgressBar(self.layoutWidget)
        self.hipADCProgress.setMaximum(65535)
        self.hipADCProgress.setProperty("value", 6500)
        self.hipADCProgress.setObjectName("hipADCProgress")
        self.horizontalLayout_3.addWidget(self.hipADCProgress)
        self.layoutWidget1 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget1.setGeometry(QtCore.QRect(570, 200, 142, 27))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_4.addWidget(self.label_2)
        self.thighADCProgress = QtWidgets.QProgressBar(self.layoutWidget1)
        self.thighADCProgress.setMaximum(65535)
        self.thighADCProgress.setProperty("value", 6500)
        self.thighADCProgress.setObjectName("thighADCProgress")
        self.horizontalLayout_4.addWidget(self.thighADCProgress)
        self.layoutWidget2 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget2.setGeometry(QtCore.QRect(570, 230, 138, 27))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget2)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_5.addWidget(self.label_3)
        self.kneeADCProgress = QtWidgets.QProgressBar(self.layoutWidget2)
        self.kneeADCProgress.setMaximum(65535)
        self.kneeADCProgress.setProperty("value", 6500)
        self.kneeADCProgress.setObjectName("kneeADCProgress")
        self.horizontalLayout_5.addWidget(self.kneeADCProgress)
        self.layoutWidget3 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget3.setGeometry(QtCore.QRect(570, 10, 148, 29))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.horizontalLayout_71 = QtWidgets.QHBoxLayout(self.layoutWidget3)
        self.horizontalLayout_71.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_71.setObjectName("horizontalLayout_71")
        self.label_70 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_70.setObjectName("label_70")
        self.horizontalLayout_71.addWidget(self.label_70)
        self.legXLineEdit = QtWidgets.QLineEdit(self.layoutWidget3)
        self.legXLineEdit.setObjectName("legXLineEdit")
        self.horizontalLayout_71.addWidget(self.legXLineEdit)
        self.layoutWidget4 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget4.setGeometry(QtCore.QRect(570, 40, 147, 29))
        self.layoutWidget4.setObjectName("layoutWidget4")
        self.horizontalLayout_70 = QtWidgets.QHBoxLayout(self.layoutWidget4)
        self.horizontalLayout_70.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_70.setObjectName("horizontalLayout_70")
        self.label_69 = QtWidgets.QLabel(self.layoutWidget4)
        self.label_69.setObjectName("label_69")
        self.horizontalLayout_70.addWidget(self.label_69)
        self.legYLineEdit = QtWidgets.QLineEdit(self.layoutWidget4)
        self.legYLineEdit.setObjectName("legYLineEdit")
        self.horizontalLayout_70.addWidget(self.legYLineEdit)
        self.layoutWidget5 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget5.setGeometry(QtCore.QRect(570, 70, 147, 29))
        self.layoutWidget5.setObjectName("layoutWidget5")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.layoutWidget5)
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_5 = QtWidgets.QLabel(self.layoutWidget5)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_9.addWidget(self.label_5)
        self.legZLineEdit = QtWidgets.QLineEdit(self.layoutWidget5)
        self.legZLineEdit.setObjectName("legZLineEdit")
        self.horizontalLayout_9.addWidget(self.legZLineEdit)
        self.layoutWidget6 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget6.setGeometry(QtCore.QRect(570, 130, 146, 29))
        self.layoutWidget6.setObjectName("layoutWidget6")
        self.horizontalLayout_72 = QtWidgets.QHBoxLayout(self.layoutWidget6)
        self.horizontalLayout_72.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_72.setObjectName("horizontalLayout_72")
        self.label_71 = QtWidgets.QLabel(self.layoutWidget6)
        self.label_71.setObjectName("label_71")
        self.horizontalLayout_72.addWidget(self.label_71)
        self.legLLineEdit = QtWidgets.QLineEdit(self.layoutWidget6)
        self.legLLineEdit.setObjectName("legLLineEdit")
        self.horizontalLayout_72.addWidget(self.legLLineEdit)
        self.layoutWidget7 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget7.setGeometry(QtCore.QRect(570, 100, 148, 29))
        self.layoutWidget7.setObjectName("layoutWidget7")
        self.horizontalLayout_69 = QtWidgets.QHBoxLayout(self.layoutWidget7)
        self.horizontalLayout_69.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_69.setObjectName("horizontalLayout_69")
        self.label_68 = QtWidgets.QLabel(self.layoutWidget7)
        self.label_68.setObjectName("label_68")
        self.horizontalLayout_69.addWidget(self.label_68)
        self.legRLineEdit = QtWidgets.QLineEdit(self.layoutWidget7)
        self.legRLineEdit.setObjectName("legRLineEdit")
        self.horizontalLayout_69.addWidget(self.legRLineEdit)
        self.layoutWidget8 = QtWidgets.QWidget(self.legTab)
        self.layoutWidget8.setGeometry(QtCore.QRect(570, 260, 131, 27))
        self.layoutWidget8.setObjectName("layoutWidget8")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.layoutWidget8)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_4 = QtWidgets.QLabel(self.layoutWidget8)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_6.addWidget(self.label_4)
        self.calfADCProgress = QtWidgets.QProgressBar(self.layoutWidget8)
        self.calfADCProgress.setMaximum(65535)
        self.calfADCProgress.setProperty("value", 6500)
        self.calfADCProgress.setObjectName("calfADCProgress")
        self.horizontalLayout_6.addWidget(self.calfADCProgress)
        self.tabs.addTab(self.legTab, "")
        self.PIDTab = QtWidgets.QWidget()
        self.PIDTab.setObjectName("PIDTab")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.PIDTab)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.pidLineChart = LineChart(self.PIDTab)
        self.pidLineChart.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.pidLineChart.sizePolicy().hasHeightForWidth())
        self.pidLineChart.setSizePolicy(sizePolicy)
        self.pidLineChart.setObjectName("pidLineChart")
        self.verticalLayout_6.addWidget(self.pidLineChart)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_29 = QtWidgets.QLabel(self.PIDTab)
        self.label_29.setObjectName("label_29")
        self.verticalLayout_5.addWidget(self.label_29)
        self.horizontalLayout_24 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_24.setObjectName("horizontalLayout_24")
        self.label_30 = QtWidgets.QLabel(self.PIDTab)
        self.label_30.setObjectName("label_30")
        self.horizontalLayout_24.addWidget(self.label_30)
        self.pidPSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidPSpin.setDecimals(4)
        self.pidPSpin.setSingleStep(0.1)
        self.pidPSpin.setObjectName("pidPSpin")
        self.horizontalLayout_24.addWidget(self.pidPSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_24)
        self.horizontalLayout_25 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_25.setObjectName("horizontalLayout_25")
        self.label_31 = QtWidgets.QLabel(self.PIDTab)
        self.label_31.setObjectName("label_31")
        self.horizontalLayout_25.addWidget(self.label_31)
        self.pidISpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidISpin.setDecimals(4)
        self.pidISpin.setSingleStep(0.1)
        self.pidISpin.setObjectName("pidISpin")
        self.horizontalLayout_25.addWidget(self.pidISpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_25)
        self.horizontalLayout_26 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_26.setObjectName("horizontalLayout_26")
        self.label_32 = QtWidgets.QLabel(self.PIDTab)
        self.label_32.setObjectName("label_32")
        self.horizontalLayout_26.addWidget(self.label_32)
        self.pidDSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidDSpin.setDecimals(4)
        self.pidDSpin.setSingleStep(0.1)
        self.pidDSpin.setObjectName("pidDSpin")
        self.horizontalLayout_26.addWidget(self.pidDSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_26)
        self.horizontalLayout_75 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_75.setObjectName("horizontalLayout_75")
        self.label_73 = QtWidgets.QLabel(self.PIDTab)
        self.label_73.setObjectName("label_73")
        self.horizontalLayout_75.addWidget(self.label_73)
        self.pidMinSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidMinSpin.setDecimals(2)
        self.pidMinSpin.setMinimum(-8192.0)
        self.pidMinSpin.setMaximum(8192.0)
        self.pidMinSpin.setSingleStep(1.0)
        self.pidMinSpin.setObjectName("pidMinSpin")
        self.horizontalLayout_75.addWidget(self.pidMinSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_75)
        self.horizontalLayout_74 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_74.setObjectName("horizontalLayout_74")
        self.label_72 = QtWidgets.QLabel(self.PIDTab)
        self.label_72.setObjectName("label_72")
        self.horizontalLayout_74.addWidget(self.label_72)
        self.pidMaxSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidMaxSpin.setDecimals(2)
        self.pidMaxSpin.setMinimum(-8192.0)
        self.pidMaxSpin.setMaximum(8192.0)
        self.pidMaxSpin.setSingleStep(1.0)
        self.pidMaxSpin.setObjectName("pidMaxSpin")
        self.horizontalLayout_74.addWidget(self.pidMaxSpin)
        self.verticalLayout_5.addLayout(self.horizontalLayout_74)
        spacerItem = QtWidgets.QSpacerItem(20, 0, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.line = QtWidgets.QFrame(self.PIDTab)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout_2.addWidget(self.line)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_25 = QtWidgets.QLabel(self.PIDTab)
        self.label_25.setObjectName("label_25")
        self.verticalLayout_4.addWidget(self.label_25)
        self.horizontalLayout_21 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_21.setObjectName("horizontalLayout_21")
        self.label_26 = QtWidgets.QLabel(self.PIDTab)
        self.label_26.setObjectName("label_26")
        self.horizontalLayout_21.addWidget(self.label_26)
        self.extendMinSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.extendMinSpin.setDecimals(0)
        self.extendMinSpin.setMaximum(8192.0)
        self.extendMinSpin.setObjectName("extendMinSpin")
        self.horizontalLayout_21.addWidget(self.extendMinSpin)
        self.extendMaxSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.extendMaxSpin.setDecimals(0)
        self.extendMaxSpin.setMaximum(8192.0)
        self.extendMaxSpin.setObjectName("extendMaxSpin")
        self.horizontalLayout_21.addWidget(self.extendMaxSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_21)
        self.horizontalLayout_23 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_23.setObjectName("horizontalLayout_23")
        self.label_28 = QtWidgets.QLabel(self.PIDTab)
        self.label_28.setObjectName("label_28")
        self.horizontalLayout_23.addWidget(self.label_28)
        self.retractMinSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.retractMinSpin.setDecimals(0)
        self.retractMinSpin.setMaximum(8192.0)
        self.retractMinSpin.setObjectName("retractMinSpin")
        self.horizontalLayout_23.addWidget(self.retractMinSpin)
        self.retractMaxSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.retractMaxSpin.setDecimals(0)
        self.retractMaxSpin.setMaximum(8192.0)
        self.retractMaxSpin.setObjectName("retractMaxSpin")
        self.horizontalLayout_23.addWidget(self.retractMaxSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_23)
        self.label_27 = QtWidgets.QLabel(self.PIDTab)
        self.label_27.setObjectName("label_27")
        self.verticalLayout_4.addWidget(self.label_27)
        self.horizontalLayout_27 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_27.setObjectName("horizontalLayout_27")
        self.label_33 = QtWidgets.QLabel(self.PIDTab)
        self.label_33.setObjectName("label_33")
        self.horizontalLayout_27.addWidget(self.label_33)
        self.seedFutureSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.seedFutureSpin.setDecimals(0)
        self.seedFutureSpin.setMaximum(8192.0)
        self.seedFutureSpin.setObjectName("seedFutureSpin")
        self.horizontalLayout_27.addWidget(self.seedFutureSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_27)
        self.horizontalLayout_28 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_28.setObjectName("horizontalLayout_28")
        self.label_34 = QtWidgets.QLabel(self.PIDTab)
        self.label_34.setObjectName("label_34")
        self.horizontalLayout_28.addWidget(self.label_34)
        self.pidErrorThresholdSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.pidErrorThresholdSpin.setDecimals(0)
        self.pidErrorThresholdSpin.setMaximum(65535.0)
        self.pidErrorThresholdSpin.setSingleStep(1.0)
        self.pidErrorThresholdSpin.setObjectName("pidErrorThresholdSpin")
        self.horizontalLayout_28.addWidget(self.pidErrorThresholdSpin)
        self.verticalLayout_4.addLayout(self.horizontalLayout_28)
        spacerItem1 = QtWidgets.QSpacerItem(20, 13, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem1)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.line_2 = QtWidgets.QFrame(self.PIDTab)
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.horizontalLayout_2.addWidget(self.line_2)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_12 = QtWidgets.QLabel(self.PIDTab)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_2.addWidget(self.label_12)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_11 = QtWidgets.QLabel(self.PIDTab)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_11.addWidget(self.label_11)
        self.adcLimitMinSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.adcLimitMinSpin.setDecimals(0)
        self.adcLimitMinSpin.setMaximum(65535.0)
        self.adcLimitMinSpin.setObjectName("adcLimitMinSpin")
        self.horizontalLayout_11.addWidget(self.adcLimitMinSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_10 = QtWidgets.QLabel(self.PIDTab)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_10.addWidget(self.label_10)
        self.adcLimitMaxSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.adcLimitMaxSpin.setDecimals(0)
        self.adcLimitMaxSpin.setMaximum(65535.0)
        self.adcLimitMaxSpin.setObjectName("adcLimitMaxSpin")
        self.horizontalLayout_10.addWidget(self.adcLimitMaxSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_10)
        self.label_13 = QtWidgets.QLabel(self.PIDTab)
        self.label_13.setObjectName("label_13")
        self.verticalLayout_2.addWidget(self.label_13)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_14 = QtWidgets.QLabel(self.PIDTab)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_12.addWidget(self.label_14)
        self.ditherTimeSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.ditherTimeSpin.setDecimals(0)
        self.ditherTimeSpin.setMaximum(1000000.0)
        self.ditherTimeSpin.setObjectName("ditherTimeSpin")
        self.horizontalLayout_12.addWidget(self.ditherTimeSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_15 = QtWidgets.QLabel(self.PIDTab)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_13.addWidget(self.label_15)
        self.ditherAmpSpin = QtWidgets.QDoubleSpinBox(self.PIDTab)
        self.ditherAmpSpin.setDecimals(0)
        self.ditherAmpSpin.setMaximum(8192.0)
        self.ditherAmpSpin.setObjectName("ditherAmpSpin")
        self.horizontalLayout_13.addWidget(self.ditherAmpSpin)
        self.verticalLayout_2.addLayout(self.horizontalLayout_13)
        spacerItem2 = QtWidgets.QSpacerItem(20, 0, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem2)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.verticalLayout_6.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_29 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_29.setObjectName("horizontalLayout_29")
        self.pidCommitButton = QtWidgets.QPushButton(self.PIDTab)
        self.pidCommitButton.setObjectName("pidCommitButton")
        self.horizontalLayout_29.addWidget(self.pidCommitButton)
        self.pidJointCombo = QtWidgets.QComboBox(self.PIDTab)
        self.pidJointCombo.setObjectName("pidJointCombo")
        self.pidJointCombo.addItem("")
        self.pidJointCombo.addItem("")
        self.pidJointCombo.addItem("")
        self.horizontalLayout_29.addWidget(self.pidJointCombo)
        self.verticalLayout_6.addLayout(self.horizontalLayout_29)
        self.tabs.addTab(self.PIDTab, "")
        self.bodyTab = QtWidgets.QWidget()
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bodyTab.sizePolicy().hasHeightForWidth())
        self.bodyTab.setSizePolicy(sizePolicy)
        self.bodyTab.setObjectName("bodyTab")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.bodyTab)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.splitter = QtWidgets.QSplitter(self.bodyTab)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.splitter.sizePolicy().hasHeightForWidth())
        self.splitter.setSizePolicy(sizePolicy)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.bodyDisplay = BodyDisplay(self.splitter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bodyDisplay.sizePolicy().hasHeightForWidth())
        self.bodyDisplay.setSizePolicy(sizePolicy)
        self.bodyDisplay.setMinimumSize(QtCore.QSize(0, 0))
        self.bodyDisplay.setObjectName("bodyDisplay")
        self.horizontalLayout_14.addWidget(self.splitter)
        self.tabs.addTab(self.bodyTab, "")
        self.horizontalLayout.addWidget(self.tabs)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.estopLabel = QtWidgets.QLabel(self.centralwidget)
        self.estopLabel.setObjectName("estopLabel")
        self.horizontalLayout_8.addWidget(self.estopLabel)
        self.heightLabel = QtWidgets.QLabel(self.centralwidget)
        self.heightLabel.setObjectName("heightLabel")
        self.horizontalLayout_8.addWidget(self.heightLabel)
        self.pressureLabel = QtWidgets.QLabel(self.centralwidget)
        self.pressureLabel.setObjectName("pressureLabel")
        self.horizontalLayout_8.addWidget(self.pressureLabel)
        self.rpmLabel = QtWidgets.QLabel(self.centralwidget)
        self.rpmLabel.setObjectName("rpmLabel")
        self.horizontalLayout_8.addWidget(self.rpmLabel)
        self.oilTempLabel = QtWidgets.QLabel(self.centralwidget)
        self.oilTempLabel.setObjectName("oilTempLabel")
        self.horizontalLayout_8.addWidget(self.oilTempLabel)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        self.legsMenu = QtWidgets.QMenu(self.menubar)
        self.legsMenu.setObjectName("legsMenu")
        self.modesMenu = QtWidgets.QMenu(self.menubar)
        self.modesMenu.setObjectName("modesMenu")
        self.calibrationMenu = QtWidgets.QMenu(self.menubar)
        self.calibrationMenu.setObjectName("calibrationMenu")
        self.configurationMenu = QtWidgets.QMenu(self.menubar)
        self.configurationMenu.setObjectName("configurationMenu")
        MainWindow.setMenuBar(self.menubar)
        self.menubar.addAction(self.legsMenu.menuAction())
        self.menubar.addAction(self.modesMenu.menuAction())
        self.menubar.addAction(self.calibrationMenu.menuAction())
        self.menubar.addAction(self.configurationMenu.menuAction())

        self.retranslateUi(MainWindow)
        self.tabs.setCurrentIndex(2)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Hip"))
        self.hipADCProgress.setFormat(_translate("MainWindow", "%v"))
        self.label_2.setText(_translate("MainWindow", "Thigh"))
        self.thighADCProgress.setFormat(_translate("MainWindow", "%v"))
        self.label_3.setText(_translate("MainWindow", "Knee"))
        self.kneeADCProgress.setFormat(_translate("MainWindow", "%v"))
        self.label_70.setText(_translate("MainWindow", "X"))
        self.label_69.setText(_translate("MainWindow", "Y"))
        self.label_5.setText(_translate("MainWindow", "Z"))
        self.label_71.setText(_translate("MainWindow", "L"))
        self.label_68.setText(_translate("MainWindow", "R"))
        self.label_4.setText(_translate("MainWindow", "Calf"))
        self.calfADCProgress.setFormat(_translate("MainWindow", "%v"))
        self.tabs.setTabText(self.tabs.indexOf(self.legTab), _translate("MainWindow", "Leg"))
        self.label_29.setText(_translate("MainWindow", "PID"))
        self.label_30.setText(_translate("MainWindow", "P"))
        self.label_31.setText(_translate("MainWindow", "I"))
        self.label_32.setText(_translate("MainWindow", "D"))
        self.label_73.setText(_translate("MainWindow", "Min"))
        self.label_72.setText(_translate("MainWindow", "Max"))
        self.label_25.setText(_translate("MainWindow", "PWM"))
        self.label_26.setText(_translate("MainWindow", "Extend"))
        self.label_28.setText(_translate("MainWindow", "Retract"))
        self.label_27.setText(_translate("MainWindow", "Misc"))
        self.label_33.setText(_translate("MainWindow", "PID future time"))
        self.label_34.setText(_translate("MainWindow", "Error Threshold"))
        self.label_12.setText(_translate("MainWindow", "ADC limits"))
        self.label_11.setText(_translate("MainWindow", "Min"))
        self.label_10.setText(_translate("MainWindow", "Max"))
        self.label_13.setText(_translate("MainWindow", "Dither"))
        self.label_14.setText(_translate("MainWindow", "Time(us)"))
        self.label_15.setText(_translate("MainWindow", "Amplitude"))
        self.pidCommitButton.setText(_translate("MainWindow", "Commit"))
        self.pidJointCombo.setItemText(0, _translate("MainWindow", "Hip"))
        self.pidJointCombo.setItemText(1, _translate("MainWindow", "Thigh"))
        self.pidJointCombo.setItemText(2, _translate("MainWindow", "Knee"))
        self.tabs.setTabText(self.tabs.indexOf(self.PIDTab), _translate("MainWindow", "PID"))
        self.tabs.setTabText(self.tabs.indexOf(self.bodyTab), _translate("MainWindow", "Body"))
        self.estopLabel.setText(_translate("MainWindow", "Estop: ?"))
        self.heightLabel.setText(_translate("MainWindow", "Height: ?"))
        self.pressureLabel.setText(_translate("MainWindow", "PSI: ?"))
        self.rpmLabel.setText(_translate("MainWindow", "RPM: ?"))
        self.oilTempLabel.setText(_translate("MainWindow", "Temp: ?"))
        self.legsMenu.setTitle(_translate("MainWindow", "Legs"))
        self.modesMenu.setTitle(_translate("MainWindow", "Modes"))
        self.calibrationMenu.setTitle(_translate("MainWindow", "Calibration"))
        self.configurationMenu.setTitle(_translate("MainWindow", "Configuration"))

from stompy.ui.nogl import LegDisplay, BodyDisplay, LineChart

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
