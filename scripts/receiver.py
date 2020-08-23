#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

if __name__  == '__main__':
    # Init ROS Node for receiver
    rospy.init_node("receiver")
    
    # Set the publisher for the image and image/compressed
    image_publisher = rospy.Publisher("/makarax/image", Image, queue_size=8)
    compressed_image_publisher = rospy.Publisher("/makarax/image/compressed", CompressedImage, queue_size=8)
    
    # Capture the video from the camera
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    # While ROS still active
    while not rospy.is_shutdown():
        # Read the frame
        ret, frame = cap.read()
        
        # Set the compressed image
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = rospy.Time.now()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()

        # Convert ROS Image to OpenCV image.
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publish the image and compressed image
        image_publisher.publish(image_msg)
        compressed_image_publisher.publish(compressed_image_msg)