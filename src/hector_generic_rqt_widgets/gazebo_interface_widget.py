#!/usr/bin/env python

import os

import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from std_srvs.srv import Empty
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties


class GazeboInterfacePlugin(Plugin):

    def __init__(self, context):
        super(GazeboInterfacePlugin, self).__init__(context)
        self_context = context

        self._widget = GazeboWidget(self, context)

        if self_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self_context.serial_number()))

        self_context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class GazeboWidget(QWidget):

    def __init__(self, parent, context):
        super(GazeboWidget, self).__init__()

        # load from ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('hector_generic_rqt_widgets'), 'resource', 'gazebo_interface.ui')
        loadUi(ui_file, self, {'QWidget': QWidget})
        self.setObjectName('GazeboInterfaceUi')

        # Qt signals
        self.simSpeedHorizontalSlider.valueChanged[int].connect(self._set_sim_speed_gazebo)
        self.unpausePushButton.clicked[bool].connect(self._unpause_gazebo)
        self.pausePushButton.clicked[bool].connect(self._pause_gazebo)
        self.resetPushButton.clicked[bool].connect(self._reset_gazebo)

    def shutdown_plugin(self):
        rospy.loginfo('Shutting down ...')
        rospy.loginfo('Done!')

    def _set_sim_speed_gazebo(self, speed):
        try:
            rospy.wait_for_service('/gazebo/get_physics_properties', 1.0)
            get_physics_prop = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
            physics = get_physics_prop()

            dt = physics.time_step * (100.0 / speed)

            rospy.wait_for_service('/gazebo/set_physics_properties', 1.0)
            set_physics_prop = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
            set_physics_prop(time_step=dt)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        except rospy.ROSException as exc:
            rospy.logerr("Service not available: " + str(exc))

    def _unpause_gazebo(self):
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', 1.0)
            unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause_physics()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        except rospy.ROSException as exc:
            rospy.logerr("Service not available: " + str(exc))

    def _pause_gazebo(self):
        try:
            rospy.wait_for_service('/gazebo/pause_physics', 1.0)
            pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause_physics()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        except rospy.ROSException as exc:
            rospy.logerr("Service not available: " + str(exc))

    def _reset_gazebo(self):
        try:
            rospy.wait_for_service('/gazebo/reset_world', 1.0)
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        except rospy.ROSException as exc:
            rospy.logerr("Service not available: " + str(exc))
