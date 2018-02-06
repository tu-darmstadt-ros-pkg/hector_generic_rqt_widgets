#!/usr/bin/env python

import os

import rospy
import rospkg

import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class DiffDriveControlPlugin(Plugin):

    def __init__(self, context):
        super(DiffDriveControlPlugin, self).__init__(context)
        self_context = context

        self._widget = DiffDriveControlWidget(self, context)

        if self_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self_context.serial_number()))

        self_context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class DiffDriveControlWidget(QWidget):

    updateLinearGoalState = pyqtSignal(float)
    updateAngularGoalState = pyqtSignal(float)
    updateLinearCurrentState = pyqtSignal(str)
    updateAngularCurrentState = pyqtSignal(str)

    def __init__(self, parent, context):
        super(DiffDriveControlWidget, self).__init__()

        self._max_linear_vel = rospy.get_param('~max_linear_vel',  0.17)
        self._max_angular_vel = rospy.get_param('~max_angular_vel', 2.0)

        self.twist_msg = Twist()

        # load from ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('hector_generic_rqt_widgets'), 'resource', 'diff_drive_control.ui')
        loadUi(ui_file, self, {'QWidget': QWidget})
        self.setObjectName('DiffDriveControlUi')

        # init elements
        self.linearGoalDoubleSpinBox.setMinimum(-self._max_linear_vel)
        self.linearGoalDoubleSpinBox.setMaximum(self._max_linear_vel)
        self.angularGoalDoubleSpinBox.setMinimum(-self._max_angular_vel)
        self.angularGoalDoubleSpinBox.setMaximum(self._max_angular_vel)

        # connect to signals
        self.linearVerticalSlider.valueChanged[int].connect(
            lambda value: self.linearGoalDoubleSpinBox.setValue(float(self._max_linear_vel * value / 100.0)))
        self.linearGoalDoubleSpinBox.valueChanged[float].connect(
            lambda value: self.linearVerticalSlider.setValue(int(value / self._max_linear_vel * 100.0)))
        self.linearGoalDoubleSpinBox.valueChanged[float].connect(
            lambda value: self._linear_cmd_update(value, 'ui'))

        self.angularDial.valueChanged[int].connect(
            lambda value: self.angularGoalDoubleSpinBox.setValue(float(self._max_angular_vel * -value / 100.0)))
        self.angularGoalDoubleSpinBox.valueChanged[float].connect(
            lambda value: self.angularDial.setValue(int(-value / self._max_angular_vel * 100.0)))
        self.angularGoalDoubleSpinBox.valueChanged[float].connect(
            lambda value: self._angular_cmd_update(value, 'ui'))

        self.stopPushButton.clicked[bool].connect(lambda: self.linearVerticalSlider.setValue(0))
        self.stopPushButton.clicked[bool].connect(lambda: self.angularDial.setValue(0))

        # Qt signals
        self.updateLinearGoalState.connect(lambda cmd: self._linear_cmd_update(cmd, 'msg'))
        self.updateAngularGoalState.connect(lambda cmd: self._angular_cmd_update(cmd, 'msg'))
        self.updateLinearCurrentState.connect(self.linearCurrentLineEdit.setText)
        self.updateAngularCurrentState.connect(self.angularCurrentLineEdit.setText)

        # init subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self._odom_cb)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_cb)

        # init publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def shutdown_plugin(self):
        rospy.loginfo('Shutting down ...')
        self.odom_sub.unregister()
        self.cmd_vel_sub.unregister()
        rospy.loginfo('Done!')

    def _linear_cmd_update(self, cmd, source):
        if source == 'ui':
            self.twist_msg.linear.x = cmd
            self.cmd_vel_pub.publish(self.twist_msg)
        if source == 'msg':
            self.linearVerticalSlider.blockSignals(True)
            self.linearVerticalSlider.setValue(int(cmd / self._max_linear_vel * 100.0))
            self.linearVerticalSlider.blockSignals(False)
            self.linearGoalDoubleSpinBox.blockSignals(True)
            self.linearGoalDoubleSpinBox.setValue(cmd)
            self.linearGoalDoubleSpinBox.blockSignals(False)

    def _angular_cmd_update(self, cmd, source):
        if source == 'ui':
            self.twist_msg.angular.z = cmd
            self.cmd_vel_pub.publish(self.twist_msg)
        if source == 'msg':
            self.angularDial.blockSignals(True)
            self.angularDial.setValue(int(-cmd / self._max_angular_vel * 100.0))
            self.angularDial.blockSignals(False)
            self.angularGoalDoubleSpinBox.blockSignals(True)
            self.angularGoalDoubleSpinBox.setValue(cmd)
            self.angularGoalDoubleSpinBox.blockSignals(False)
        
    def _cmd_vel_cb(self, msg):
        if msg._connection_header['callerid'] != rospy.get_name():
            self.twist_msg = msg
            self.updateLinearGoalState.emit(msg.linear.x)
            self.updateAngularGoalState.emit(msg.angular.z)

    def _odom_cb(self, odom):
        self.updateLinearCurrentState.emit(str('%.2f' % odom.twist.twist.linear.x))
        self.updateAngularCurrentState.emit(str('%.2f' % odom.twist.twist.angular.z))

