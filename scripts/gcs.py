#!usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import cv2 as cv
import numpy
import rospy
import Tkinter
from PIL import Image, ImageTk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16
# Local Import
from kkctbn2020.msg import AutoControl, Config, ObjectCount, Mode

ori = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
red_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
green_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
throttle_pwm = 0
auto_ctrl = "NONE"
mode = "NONE"

def add_slider(text, from_, to_, resolution, master, default=0):
    frame = Tkinter.Frame(master=master)
    label = Tkinter.Label(frame, text=text, fg='black', font=("Helvetica"))
    label.grid(row=1, column=2, padx=10, pady=0)
    scale = Tkinter.Scale(frame, from_=from_, to_=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, length=300)
    scale.set(default)
    scale.grid(row=1, column=2, padx=10, pady=0)
    frame.pack()
    return scale

def image_callback(img):
    global ori
    ori_cv = numpy.fromstring(img.data, numpy.uint8)
    ori_cv = cv.imdecode(ori_cv, cv.IMREAD_COLOR)
    ori = ori_cv

def red_mask_callback(img):
    global red_mask
    mask_cv = numpy.fromstring(img.data, numpy.uint8)
    mask_cv = numpy.imdecode(mask_cv, cv.IMREAD_COLOR)
    red_mask = mask_cv

def green_mask_callback(img):
    global green_mask
    mask_cv = numpy.fromstring(img.data, numpy.uint8)
    mask_cv = numpy.imdecode(mask_cv, cv.IMREAD_COLOR)
    green_mask = mask_cv

def throttle_pwm_callback(pwm):
    global throttle_pwm
    throttle_pwm = pwm.data

def mode_callback(mode_in):
    global mode
    if mode_in.value == Mode.ARMED:
        mode = "ARMED"
    elif mode_in.value == Mode.HOLD:
        mode = "HOLD"

def auto_control_callback(msg):
    global auto_ctrl
    if msg.state == AutoControl.AVOID_RED:
        auto_ctrl = "AVOID RED"
    elif msg.state == AutoControl.AVOID_RED_AND_GREEN:
        auto_ctrl = "AVOID RED AND GREEN"
    else:
        auto_ctrl = "AVOID WHAT"

if __name__ == '__main__':
    # Initialize gcs node
    rospy.init_node('gcs', anonymous=True)

    # Set Publisher nodes
    cfg_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)

    # Set Subscriber nodes
    img_subscriber = rospy.Subscriber("/makarax/image/processed/compressed", CompressedImage, image_callback)
    auto_ctrl_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    red_mask_subscriber = rospy.Subscriber("/makarax/image/mask/red/compressed", CompressedImage, red_mask_callback)
    green_mask_subscriber = rospy.Subscriber("/makarax/image/mask/green/compressed", CompressedImage, green_mask_callback)
    throttle_pwm_subscriber = rospy.Subscriber("/makarax/pwm_throttle", UInt16, throttle_pwm_callback)
    mode_subscriber = rospy.Subscriber("/makarax/mode", Mode, mode_callback)

    master = Tkinter.Tk()
    master.title("Config")

    slider_frame1 = Tkinter.Frame(master=master)

    contrast = add_slider('Contrast', -255, 255, 1, slider_frame1, 0)
    brightness = add_slider('Brightness', -127, 127, 1, slider_frame1, -2)
    gamma = add_slider('Gamma', 0.1, 3, 0.1, slider_frame1, 1)
    roi_y = add_slider('ROI Y', 0, 480, 1, slider_frame1, 5)
    red_low_hue = add_slider('RED L-HUE', 0, 255, 1, slider_frame1, 118)
    red_low_sat = add_slider('RED L-SAT', 0, 255, 1, slider_frame1, 77)
    red_low_val = add_slider('RED L-VAL', 0, 255, 1, slider_frame1, 0)
    red_high_hue = add_slider('RED H-HUE', 0, 255, 1, slider_frame1, 186)
    red_high_sat = add_slider('RED H-SAT', 0, 255, 1, slider_frame1, 255)
    red_high_val = add_slider('RED H-VAL', 0, 255, 1, slider_frame1, 255)

    slider_frame2 = Tkinter.Frame(master=master)
    green_low_hue = add_slider('GREEN L-HUE', 0, 255, 1, slider_frame1, 69)
    green_low_sat = add_slider('GREEN L-SAT', 0, 255, 1, slider_frame1, 43)
    green_low_val = add_slider('GREEN L-VAL', 0, 255, 1, slider_frame1, 0)
    green_high_hue = add_slider('GREEN H-HUE', 0, 255, 1, slider_frame1, 99)
    green_high_sat = add_slider('GREEN H-SAT', 0, 255, 1, slider_frame1, 255)
    green_high_val = add_slider('GREEN H-VAL', 0, 255, 1, slider_frame1, 255)

    slider_frame1.grid(row=1, column=2)
    slider_frame2.grid(row=1, column=3)

    info_frame = Tkinter.Frame(master=master)
    pwm_label = Tkinter.Label(info_frame, text="PWM Throttle: " + str(throttle_pwm), fg='black', font=("Helvetica", 12))
    pwm_label.pack()

    mode_label = Tkinter.Label(info_frame, text="Mode: " + mode, fg='black', font=("Helvetica", 12))
    mode_label.pack()

    auto_label = Tkinter.Label(info_frame, text="Auto: " + auto_ctrl, fg='black', font=("Helvetica", 12))
    auto_label.pack()

    info_frame.grid(row=1, column=1)

    ori_label = Tkinter.Label(master=master, image=None)
    ori_label.grid(row=2, column=1)

    red_mask_label = Tkinter.Label(master=master, image=None)
    red_mask_label.grid(row=2, column=2)

    green_mask_label = Tkinter.Label(master=master, image=None)
    green_mask_label.grid(row=2, column=3)

    while not rospy.is_shutdown():
        if ori is not None:
            b, g, r = cv.split(ori)
            img_array = cv.merge(r, g, b)
            img = Image.fromarray(img_array)
            img_tk = ImageTk.PhotoImage(image=img)
            ori_label.config(image=img_tk)

        if red_mask is not None:
            b, g, r = cv.split(red_mask)
            img_array = cv.merge(r, g, b)
            img = Image.fromarray(img_array)
            img_tk = ImageTk.PhotoImage(image=img)
            red_mask_label.config(image=img_tk)

        if green_mask is not None:
            b, g, r = cv.split(green_mask)
            img_array = cv.merge(r, g, b)
            img = Image.fromarray(img_array)
            img_tk = ImageTk.PhotoImage(image=img)
            green_mask_label.config(image=img_tk)

        pwm_label.config(text="PWM Throttle: " + str(throttle_pwm))
        mode_label.config(text="Mode: " + mode)
        auto_label.config(text="Auto: " + auto_ctrl)
        master.update()

        cfg = config()
        cfg.red_low_hue = red_low_hue.get()
        cfg.red_low_sat = red_low_sat.get()
        cfg.red_low_val = red_low_val.get()
        cfg.red_high_hue = red_high_hue.get()
        cfg.red_high_sat = red_high_sat.get()
        cfg.red_high_val = red_high_val.get()
        cfg.green_low_hue = green_low_hue.get()
        cfg.green_low_sat = green_low_sat.get()
        cfg.green_low_val = green_low_val.get()
        cfg.green_high_hue = green_high_hue.get()
        cfg.green_high_sat = green_high_sat.get()
        cfg.green_high_val = green_high_val.get()
        cfg.brightness = brightness.get()
        cfg.contrast = contrast.get()
        cfg.gamma = gamma.get()
        cfg.roi_y = roi_y.get()

        config_publisher.publish(msg)
