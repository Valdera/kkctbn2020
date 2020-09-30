#!/usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import rospy
from std_msgs.msg import Float64
from kkctbn2020.msg import AutoControl


def autoControlCallback(msg):
    # Check if ROS Publisher is still active
    if (msg.state == AutoControl.AVOID_RED_AND_GREEN):
        setpoint = 640 / 2
    else:
        setpoint = -20
    
    setpoint_node.publish(setpoint)

if __name__ == '__main__':
    # If ROS isn't active, do nothing
    rospy.init_node('setpoint', anonymous=True)
    setpoint_node = rospy.Publisher("setpoint", Float64, queue_size=1)
    auto_control_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, autoControlCallback)
    
    rospy.spin()

    
