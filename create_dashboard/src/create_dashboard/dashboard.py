import roslib;roslib.load_manifest('create_dashboard')
import rospy

import diagnostic_msgs
import create_node.srv
import create_node.msg

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, BatteryDashWidget, IconToolButton, NavViewDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize

from .battery import TurtlebotBattery
from linux_hardware.msg import LaptopChargeStatus

import rospkg
import os.path

rp = rospkg.RosPack()

image_path = image_path = os.path.join(rp.get_path('create_dashboard'), 'images')

class BreakerButton(IconToolButton):
    def __init__(self, name, onclick):
        self._on_icon = ['bg-green.svg', 'ic-breaker.svg']
        self._off_icon = ['bg-red.svg', 'ic-breaker.svg']
    
        icons = [self._on_icon, self._off_icon]

        super(BreakerButton, self).__init__(name, icons=icons)

        self.setFixedSize(self._icons[0].actualSize(QSize(50,30)))

        self.clicked.connect(onclick)

class CreateDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0,0,0]

        # These were moved out of get_widgets because they are sometimes not defined
        # before being used by dashboard_callback. Could be done more cleanly than this
        # though.
        self.lap_bat = BatteryDashWidget("Laptop")
        self.create_bat = BatteryDashWidget("Create")
        self.breakers = [BreakerButton('breaker0', lambda: self.toggle_breaker(0)), 
                         BreakerButton('breaker1', lambda: self.toggle_breaker(1)),
                         BreakerButton('breaker2', lambda: self.toggle_breaker(2))]
        
        # This is what gets dashboard_callback going eagerly
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
        self._power_control = rospy.ServiceProxy('turtlebot_node/set_digital_outputs', create_node.srv.SetDigitalOutputs)
        self._laptop_bat_sub = rospy.Subscriber('/laptop_charge', LaptopChargeStatus, self.laptop_cb)

    def get_widgets(self):
        self.mode = MenuDashWidget('Mode')

        self.mode.add_action('Full', self.on_full_mode)
        self.mode.add_action('Passive', self.on_passive_mode)
        self.mode.add_action('Safe', self.on_safe_mode)

        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context), self.mode],
                self.breakers,
                [self.lap_bat, self.create_bat],
                [NavViewDashWidget(self.context)]]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()
  
        battery_status = {}
        laptop_battery_status = {}
        breaker_status = {}
        op_mode = None
        for status in msg.status:
            if status.name == "/Power System/Battery":
                for value in status.values:
                    if value.key == 'Percent':
                        self.create_bat.update_perc(float(value.value))
                        self.create_bat.update_time(float(value.value))
                    elif value.key == "Charging State":
                        if value.value == "Trickle Charging" or value.value == "Full Charging":
                            self.create_bat.set_charging(True)
                        else:
                            self.create_bat.set_charging(False)
            # Is this one needed? I don't think so.
            if status.name == "/Power System/Laptop Battery":
                for value in status.values:
                    laptop_battery_status[value.key]=value.value
            if status.name == "/Mode/Operating Mode":
                op_mode=status.message
            if status.name == "/Digital IO/Digital Outputs":
                #print "got digital IO"
                for value in status.values:
                    breaker_status[value.key]=value.value
  
        #########################################################
        # Maybe don't need any of this battery stuff anymore?
        #########################################################
        if (battery_status):
          self.create_bat.set_power_state(battery_status)
        else:
          #self._power_state_ctrl.set_stale()
          print("Power State Stale")
  
        if (laptop_battery_status):
          self.lap_bat.set_power_state(laptop_battery_status)
        else:
          #self._power_state_ctrl_laptop.set_stale()
          print("Laptop battery stale")
        #########################################################
        
        if (breaker_status):
            self._raw_byte = int(breaker_status['Raw Byte'])
            self._update_breakers()

    def laptop_cb(self, msg):
        self.lap_bat.update_perc(float(msg.percentage))
        self.lap_bat.update_time(float(msg.percentage))
        self.lap_bat.set_charging(bool(msg.charge_state))

    def toggle_breaker(self, index):
        try:
            self.digital_outs[index] = not self.digital_outs[index]
            power_cmd = create_node.srv.SetDigitalOutputsRequest(self.digital_outs[0], self.digital_outs[1], self.digital_outs[2])
            #print power_cmd
            self._power_control(power_cmd)

            return True
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Service call failed with error: %s"%(e))
            self.message.exec_()
            return False
          
        return False

    def _update_breakers(self):
        tmp = self._raw_byte
        for i in range(0,3):
            self.digital_outs[i]=tmp%2
            self.breakers[i].update_state(self.digital_outs[i])
            tmp = tmp >> 1
        
    def on_passive_mode(self):
        passive = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",create_node.srv.SetTurtlebotMode )
        try:
            passive(create_node.msg.TurtlebotSensorState.OI_MODE_PASSIVE)
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Failed to put the turtlebot in passive mode: service call failed with error: %s"%(e))
            self.message.exec_() 

    def on_safe_mode(self):
        safe = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",create_node.srv.SetTurtlebotMode)
        try:
          safe(create_node.msg.TurtlebotSensorState.OI_MODE_SAFE)
        except rospy.ServiceException, e:
          self.message = QMessageBox()
          self.message.setText("Failed to put the turtlebot in safe mode: service call failed with error: %s"%(e))
          self.message.exec_()

    def on_full_mode(self):
        full = rospy.ServiceProxy("/turtlebot_node/set_operation_mode", create_node.srv.SetTurtlebotMode)
        try:
            full(create_node.msg.TurtlebotSensorState.OI_MODE_FULL)
        except rospy.ServiceException, e:
            self.message = QMessageBox()
            self.message.setText("Failed to put the turtlebot in full mode: service call failed with error: %s"%(e))
            self.message.exec_()
