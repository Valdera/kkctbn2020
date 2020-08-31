#!/usr/bin/env python

import rospy
from kkctbn2020.msg import Mode
from mavros_msgs.msg import RCIn

def pwm_mode_callback(msg):
	try:
		pwm = msg.channels[7]
	except IndexError:
		pwm = 1700
		
	mode = Mode()
	if (pwm < 1400):
		mode.value = Mode.HOLD
	elif (pwm > 1600):
		mode.value = Mode.ARMED
	
	if (pwm < 1400 or pwm > 1600):
		mode_publisher.publish(mode)
	
if __name__ == '__main__':
	rospy.init_node("mode")
	rc_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, pwm_mode_callback)
	mode_publisher = rospy.Publisher("/makarax/mode", Mode, queue_size=8)

	rospy.spin()
