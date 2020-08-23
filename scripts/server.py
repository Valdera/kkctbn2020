#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from kkctbn2020.cfg import Kkctbn2020Config

def callback(config, level):
    # Return the current configuration
    return config

if __name__ == "__main__":
    # Init ROS Node for server
    rospy.init_node("server", anonymous=False)
    
    # Configure the server and send the configuration
    server = Server(Kkctbn2020Config, callback)
    rospy.spin()
    