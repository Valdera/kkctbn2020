#!/usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import rospy
from std_msgs.msg import Float64

def setpoint():
    # initialize the node
    rospy.init_node('setpoint', anonymous=True)
    # set setpoint as ROS Publisher
    setpoint_node = rospy.Publisher("gcs", Float64, queue_size=1)

    rospy.loginfo("Starting setpoint publisher")

    # Check if ROS Publisher is still active
    # Check the time
    while ((not rospy.is_shutdown()) and (rospy.Time(0) == rospy.Time.now())):
        rospy.loginfo("setpoint_node spinning, waiting for time to become non-zero")
        rospy.sleep(1)

    # setpoint -10 as Float64 
    
    # (code in C++) Cb dicek bener ato kgk :)
    # std_msgs::Float64 setpoint
    # setpoint.data = -10
    
    setpoint = -10
    # Set the rate to 0.2 Hz which is 5 seconds
    rate = rospy.Rate(0.2)

    # Publish setpoint if ROS is still active
    while not rospy.is_shutdown():
        # Publish setpoint value
        setpoint_node.publish(setpoint)
        rate.sleep()


if __name__ == '__main__':
    # If ROS isn't active, do nothing
    try:
        setpoint()
    except rospy.ROSInterruptException:
        pass
