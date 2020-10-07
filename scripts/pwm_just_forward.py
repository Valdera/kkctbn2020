#!/usr/bin/env python
import rospy
from kkctbn2020.msg import AutoControl, Mode
from std_msgs.msg import Bool, Float64, Int16

# State Value
current_auto_control = AutoControl()
current_mode = Mode()
init_compass = None
current_compass = None
degree_decision = 95.0
lower_tolerance = None
upper_tolerance = None

def mode_callback(msg):
    global current_mode
    current_mode = msg

def auto_control_callback(msg):
    global current_auto_control
    current_auto_control = msg

# Setting Degree
def degree_decision_callback(msg):
    global degree_decision
    degree_decision = msg.data

def compass_callback(degree):
    global  init_compass, current_compass, degree_decision, upper_tolerance, lower_tolerance, current_auto_control, current_mode
    published_data = Bool()
    published_data.data = False

    current_compass = degree.data
    print(lower_tolerance)
    print(upper_toleranceprin)
    # If between the tolerance degree just push forward    
    if (current_mode.value == Mode.ARMED):
        # if on AutoControl mode, publish the data
        if (current_auto_control.state == AutoControl.AVOID_RED_AND_GREEN) or (current_auto_control.state == AutoControl.AVOID_RED):
            if init_compass is None:
                init_compass = degree.data
                # Set the tolerance degree to push forward (85 - 95)
                upper_tolerance = init_compass + degree_decision + 10.0
                lower_tolerance = init_compass + degree_decision - 10.0 
                if upper_tolerance > 360.0:
                    upper_tolerance -= 360
                    lower_tolerance -= 360
            
            if current_compass >= lower_tolerance and current_compass <= upper_tolerance:
                published_data.data = True

            pwm_just_forward_publisher.publish(published_data)
    # If on manual mode, reset everything
    else:
        init_compass = None
        published_data.data = False
        pwm_just_forward_publisher.publish(published_data)


if __name__ == '__main__':
    rospy.init_node("pwm_just_forward")

    # Publisher
    pwm_just_forward_publisher = rospy.Publisher("/makarax/pwm_just_forward", Bool, queue_size=8)

    # Subscriber
    auto_control_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    mode_subscriber = rospy.Subscriber("/makarax/mode", Mode, mode_callback)
    compass_subscriber = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, compass_callback)
    degree_decision_subscriber = rospy.Subscriber("/makarax/degree_decision", Int16, degree_decision_callback)

    rospy.spin()
