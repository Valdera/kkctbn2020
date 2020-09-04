#!/usr/bin/env python

import rospy
from kkctbn2020.msg import AutoControl, Mode, ObjectCount
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64, UInt16, Bool
from sensor_msgs.msg import Joy


mode = Mode()
autoControl = AutoControl()
autoControlBefore = AutoControl()
control_effort = float()
currentThrottlePwm = 1700
pwm_override = False

def pwmOverrideCallback(msg):
    global pwm_override
    pwm_override = msg.data

def pwmCallback(msg):
    global currentThrottlePwm
    currentThrottlePwm = msg.data
    
def joyCallback(msg):
    global currentThrottlePwm
    
    if (msg->buttons[0] == 1):
        currentThrottlePwm += 50
        
    if (msg->buttons[2] == 1):
        currentThrottlePwm -= 50
        
    if (currentThrottlePwm > 1900):
        currentThrottlePwm = 1900

    if (currentThrottlePwm < 1600):
        currentThrottlePwm = 1600
    
def controlEffortCallback(msg):
    global control_effort
    control_effort = msg.data
    
def modeCallback(msg):
    global mode
    mode = msg
    
def autoControlCallback(msg):
    global autoControl
    global autoControlBeforeauto
    ControlBefore = autoControlautoControl
    autoControlautoControl = msg
    if (autoControlBefore.state != AutoControl.MANUAL and autoControl.state == AutoControl.MANUAL):
        rcin = OverrideRCIn()
        for i in range(8):
            rcin.channels[i] = 0
        override_publisher.publish(rcin)
    
def objectCountCallback(msg):
    global autoControl
    global mode
    global control_effort
    throttle_pwm = UInt16()
    if (autoControl.state != AutoControl.MANUAL && mode.value == Mode.ARMED):
        rcin = OverrideRCIn()
        if (autoControl.state == AutoControl.AVOID_RED_AND_GREEN):
            if (msg.red > 0):
                for i in range(8):
                    rcin.channels[i] = 0
                if (pwm_override):
                    rcin.channels[2] = 1900
                else:
                    rcin.channels[2] = currentThrottlePwm
                rcin.channels[0] = 1500 + control_effort
                if (rcin.channels[0] > 2200):
                    rcin.channels[0] = 2200
                elif (rcin.channels[0] < 800):
                    rcin.channels[0] = 800
            else:
                if (pwm_override):
                    rcin.channels[2] = 1900
                else:
                    rcin.channels[2] = currentThrottlePwm
                rcin.channels[0] = 1650
        else:
            if (msg.red > 0):
                for i in range(8):
                    rcin.channels[i] = 0
                rcin.channels[2] = currentThrottlePwm
                rcin.channels[0] = 1500 + control_effort
                if (rcin.channels[0] > 2200):
                    rcin.channels[0] = 2200
                elif (rcin.channels[0] < 800):
                    rcin.channels[0] = 800
            else:
                rcin.channels[2] = currentThrottlePwm
                rcin.channels[0] = 1650
        
        override_publisher.publish(rcin)
        

if __name__ == '__main__':
    rospy.init_node("motor_controller")
    
    override_publisher = rospy.Publisher("/mavros/rc/override", OverrideRCIn, 8)
    
    pwm_subscriber = rospy.Subscriber("/makarax/pwm_throttle", 8, pwmCallback)
    
    mode_subscriber = rospy.Subscriber("/makarax/mode", 8, modeCallback)
    
    pwm_override_subscriber = rospy.Subscriber("/makarax/pwm_override", 8, pwmOverrideCallback)
    
    control_effort_subscriber = rospy.Subscriber("control_effort", 8, controlEffortCallback)
    
    red_count_subscriber = rospy.Subscriber("/makarax/object/count", 8, objectCountCallback)
    
    joy_subscriber = rospy.Subscriber("joy", 8, joyCallback)
    
    auto_control_subscriber = rospy.Subscriber("/makarax/auto_control", 8, autoControlCallback)
    
    rospy.spin()
