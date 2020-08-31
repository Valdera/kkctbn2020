#!/usr/bin/env python
import rospy
import threading
from kkctfbn2020.msg import AutoControl
from stf_msgs.msg import Bool
import dynamic_reconfigure.client

auto_control_before = AutoControl()
time = 9

def timer_action():
	'''
	publish msg True ke pwm_override_publisher
	'''
	msg = Bool()
	msg.data = True
	pwm_override_publisher.publish(msg)
	
def auto_control_callback(msg):
	'''
	publish True ke pwm_override_published setelah time detik
	sejak kali pertama mendapat sinyal AVOID_RED_AND_GREEN dari msg setelah sebelumnya tidak mendapat sinyal tersebut
	'''
	
	global auto_control_before
	published_data = Bool()
	published_data.data = False
	
	if msg.state == AutoControl.AVOID_RED_AND_GREEN and auto_control_before.state != AutoControl.AVOID_RED_AND_GREEN:
		pwm_override_publisher.publish(published_data)
		timer = threading.Timer(time, timer_action)
		timer.start()
	elif msg.state != AutoControl.AVOID_RED_AND_GREEN and auto_control_before.state == AutoControl.AVOID_RED_AND_GREEN:
		pwm_override_publisher.publish(published_data)
		
	auto_control_before = msg

def callback(config):
	'''
	mengubah waktu sesuai dengan config
	'''
	global time
	time = config.time
	
if __name__ == '__main__':
	rospyt.init_node("pwm_override")
	
	client = dynamic_reconfigure.client.Client("Server", config_callback=callback)
	
	pwm_override_publisher = rospy.Publisher("/makarax/pwm_override", Bool, queue_size=8)
	
	auto_control_publisher = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
	
	rospy.spin()
