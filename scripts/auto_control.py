#!/usr/bin/env python
import rospy
from mavros_msgs.msg import RCIn
from kkctbn2020.msg import AutoControl

def rcin_callback(msg):
    # Get the current pwm, set the pwm to 1300 if there is an IndexError
    try:
        current_pwm = msg.channels[6]
    except IndexError:
        current_pwm = 1300

    control = AutoControl()
    # Configure the correct control based on the current pwm
    if current_pwm < 1400:
        control.state = AutoControl.MANUAL
    elif current_pwm > 1600:
        # Avoid Red And Green
        control.state = AutoControl.AVOID_RED_AND_GREEN
    else:
        control.state = AutoControl.AVOID_RED

    # Publish the corresponding control
    auto_control_publisher.publish(control)

if __name__  == '__main__':
    # Init ROS Node for auto_control
    rospy.init_node("auto_control")

    # Subscribe to mavros/rc/in
    rcin_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, rcin_callback)
    # Set the Publisher for auto_control
    auto_control_publisher = rospy.Publisher("/makarax/auto_control", AutoControl, queue_size=8)
    
    rospy.spin()
