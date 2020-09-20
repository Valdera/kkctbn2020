#!/usr/bin/env python
import rospy
from kkctbn2020.msg import AutoControl
from std_msgs.msg import Bool, Float64

# State Value
current_auto_control = AutoControl()
init_compass = None
current_compass = None
degree_decision = 90.0

def auto_control_callback(msg):
    global current_auto_control
    current_auto_control = msg.state

def compass_callback(degree):
    global  init_compass, current_compass, degree_decision
    published_data = Bool()
    published_data.data = False

    current_compass = degree

    # Initialize start degree
    if init_compass is None:
        init_compass = degree
        # Set the tolerance degree to push forward (89 - 91)
        upper_tolerance = init_compass + degree_decision + 1.0
        lower_tolerance = init_compass + degree_decision - 1.0

    # If between the tolerance degree just push forward
    if current_compass >= lower_tolerance and current_compass <= upper_tolerance:
        published_data.data = True

    # if on AutoControl mode, publish the data
    if auto_control.state == AutoControl.AVOID_RED_AND_GREEN or auto_control.state == AutoControl.AVOID_RED:
        pwm_just_forward_publisher.publish(published_data)
    # If on manual mode, reset everything
    elif auto_control.state == AutoControl.MANUAL:
        init_compass = None
        published_data.data = False
        pwm_just_forward_publisher.publish(published_data)

if __name__ == '__main__':
    rospy.init_node("pwm_just_forward")

    pwm_just_forward_publisher = rospy.Publisher("/makarax/pwm_just_forward", Bool, queue_size=8)
    auto_control_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    compass_subscriber = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, compass_callback)
    rospy.spin()
