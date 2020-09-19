#!/usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import cv2 as cv
import numpy
import rospy
import dynamic_reconfigure.client
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64
# Local Import
from kkctbn2020.msg import AutoControl, Config, ObjectCount
from kkctbn2020.cfg import *

data = None
MIN_AREA = 600

# Configure the limit of each color in term of HSV
cfg = Config()
cfg.red_low_hue = 110
cfg.red_low_sat = 56
cfg.red_low_val = 0
cfg.red_high_hue = 255
cfg.red_high_sat = 255
cfg.red_high_val = 255

cfg.green_low_hue = 69
cfg.green_low_sat = 43
cfg.green_low_val = 0
cfg.green_high_hue = 99
cfg.green_high_sat = 255
cfg.green_high_val = 255

cfg.brightness = 0
cfg.contrast = 0
cfg.gamma = 0

# Configure AutoControl
auto_ctrl = AutoControl()
auto_ctrl.state = AutoControl.AVOID_RED_AND_GREEN

# define nothing
def nothing(x):
    pass

# If there is a callback,
# set the var to the global var cfg
def config_callback(cfg_in):
    global cfg
    cfg = cfg_in

# If there is a callback,
# set the var to the global var auto_ctrl
def auto_control_callback(msg):
    global auto_ctrl
    auto_ctrl.state = msg.state

# Make it if the environment is dark, the image will become brighter
def adjust_gamma(img, gamma=1.0):
    # Build a lookup table mapping the pixel values [0, 255]
    # their adjusted gamma values
    inverted = 1.0 / gamma
    table = numpy.array([((i / 255.0) ** inverted) * 255 for i in numpy.arange(256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv.LUT(img, table)

if __name__ == '__main__':
    # Initialize image_processing node
    rospy.init_node('image_processing', anonymous=True)

    # Set Publisher nodes
    object_count_publisher = rospy.Publisher('/makarax/object/count', ObjectCount, queue_size=8)
    state_publisher = rospy.Publisher('state', Float64, queue_size=8)
    processed_image_publisher = rospy.Publisher('/makarax/image/processed/compressed', CompressedImage, queue_size=8)
    red_mask_publisher = rospy.Publisher('/makarax/image/mask/red/compressed', CompressedImage, queue_size=8)
    green_mask_publisher = rospy.Publisher('/makarax/image/mask/green/compressed', CompressedImage, queue_size=8)

    # Set Subscriber nodes
    image_subscriber = rospy.Subscriber("/makarax/image", Image, nothing)
    cfg_subscriber = rospy.Subscriber("/makarax/config", Config, config_callback)
    auto_ctrl_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)

    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/makarax/image', Image)

        # do some stuff
        count_red = 0
        count_green = 0
        bridge = CvBridge()
        ori = bridge.imgmsg_to_cv2(data)

        frame = ori.copy()

        height, width = frame.shape[:2]

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 1)
        kernel = numpy.ones((3, 3), numpy.uint8)

        # Set Range of Interest (ROI)
        roi_y = cfg.roi_y
        cv.line(frame, (width, roi_y), (0, roi_y), (0, 255, 0), 2)

        # Set up Red HSV
        red_low_hue = cfg.red_low_hue
        red_low_sat = cfg.red_low_sat
        red_low_val = cfg.red_low_val
        red_high_hue = cfg.red_high_hue
        red_high_sat = cfg.red_high_sat
        red_high_val = cfg.red_high_val

        low_red = numpy.array([red_low_hue, red_low_sat, red_low_val])
        high_red = numpy.array([red_high_hue, red_high_sat, red_high_val])

        red_mask = cv.inRange(hsv, low_red, high_red)

        # Detect Contours
        contours, _ = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
        min_x = 9999
        for cnt in contours:
            area = cv.contourArea(cnt)
            approx = cv.approxPolyDP(cnt, 0.02 * cv.arcLength(cnt, True), True)
            M = cv.moments(cnt)
            if M["m00"] == 0:
                continue

            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])

            if area > MIN_AREA and y > roi_y:
                cv.drawContours(frame, [approx], 0, (0, 0, 0), 5)
                cv.circle(frame, (x, y), 5, (0, 255, 0), -1)
                if 7 <= len(approx) < 20:
                    cv.putText(frame, "Circle Red", (x, y), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
                    count_red += 1
                    if x < min_x:
                        min_x = x

        # Set up Green HSV
        green_low_hue = cfg.green_low_hue
        green_low_sat = cfg.green_low_sat
        green_low_val = cfg.green_low_val
        green_high_hue = cfg.green_high_hue
        green_high_sat = cfg.green_high_sat
        green_high_val = cfg.green_high_val

        low_green = numpy.array([green_low_hue, green_low_sat, green_low_val])
        high_green = numpy.array([green_high_hue, green_high_sat, green_high_val])

        green_mask = cv.inRange(hsv, low_green, high_green)

        # Detect Countours
        contours, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
        max_x = 0
        for cnt in contours:
            area = cv.contourArea(cnt)
            approx = cv.approxPolyDP(cnt, 0.02 * cv.arcLength(cnt, True), True)
            M = cv.moments(cnt)
            if M["m00"] == 0:
                continue

            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])

            if area > MIN_AREA and y > roi_y:
                cv.drawContours(frame, [approx], 0, (0, 0, 0), 5)
                cv.circle(frame, (x, y), 5, (0, 255, 0), -1)
                if 7 <= len(approx) < 20:
                    cv.putText(frame, "Circle green", (x, y), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
                    count_green += 1
                    if x > max_x:
                        max_x = x

        # cv.imshow("Frame", frame)
        # cv.imshow("red_mask", red_mask)
        # cv.waitKey(30)
        objectCount = ObjectCount()
        objectCount.red = count_red
        objectCount.green = count_green
        object_count_publisher.publish(objectCount)
        
        state = Float64()
        state.data = min_x
    
        if auto_ctrl.state == AutoControl.AVOID_RED_AND_GREEN and count_green > 0:
            state = Float64()
            state.data = max_x - 640

        state_publisher.publish(state)
        

        published_red_mask = CompressedImage()
        published_red_mask.header.stamp = rospy.Time.now()
        published_red_mask.format = "jpeg"
        published_red_mask.data = numpy.array(cv.imencode(".jpg", red_mask)[1]).tostring()
        red_mask_publisher.publish(published_red_mask)

        published_green_mask = CompressedImage()
        published_green_mask.header.stamp = rospy.Time.now()
        published_green_mask.format = "jpeg"
        published_green_mask.data = numpy.array(cv.imencode(".jpg", green_mask)[1]).tostring()
        green_mask_publisher.publish(published_green_mask)

        processed_image = CompressedImage()
        processed_image.header.stamp = rospy.Time.now()
        processed_image.format = "jpeg"
        processed_image.data = numpy.array(cv.imencode(".jpg", frame)[1]).tostring()
        processed_image_publisher.publish(processed_image)
