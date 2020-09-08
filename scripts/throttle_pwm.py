#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
import dynamic_reconfigure.client

# Set default pwm to 1750
pwm_value = 1750

def callback(config):
    # Reference to the global pwm
    global pwm_value

    # Set the current pwm based on the configuration 
    pwm_value = config.pwm
    current_pwm = UInt16()
    current_pwm.data = pwm_value
    
    # Publish the pwm based on the configuration from the server
    pwm_throttle_publisher.publish(current_pwm)


if __name__ == "__main__":
    # Init ROS Node for mode
    rospy.init_node("mode")

    # Subscribe to the server
    client = dynamic_reconfigure.client.Client("server", config_callback=callback)

    # Set the Publisher for pwm_throttle
    pwm_throttle_publisher = rospy.Publisher("/makarax/pwm_throttle", UInt16, queue_size=8)
    
    rospy.spin()