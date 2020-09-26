#!/usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import cv2 as cv
import numpy
import rospy
import Tkinter
from PIL import Image, ImageTk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16, Int16
# Local Import
from kkctbn2020.msg import AutoControl, Config, ObjectCount, Mode

ori = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
red_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
green_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
throttle_pwm = 0
auto_ctrl = "None"
mode = "None"
detect = "None"

pwm_input = 0
frame, threshold, image_hsv = None, None, None
mouseX, mouseY = None, None
roi = None

def nothing(num):
    pass

def add_slider(text, from_, to_, resolution, frame, row, default=0):
    label = Tkinter.Label(frame, text=text, fg='black', font=("Helvetica", 12))
    label.grid(row=row, column=0, padx=5, sticky='se', pady=2)
    scale = Tkinter.Scale(frame, from_=from_, to=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, length=300)
    scale.set(default)
    scale.grid(row=row, column=1, padx=5, sticky='n', pady=0)
    return scale

# def selectROI(event, x, y, flags, params):
#     global mouseX, mouseY, image_hsv, roi, detect #, new_img
#     if event == cv.EVENT_LBUTTONDOWN:
#         # cv.circle(new_img, (x, y), 4, (0, 255, 0), 2)
#         mouseX, mouseY = x, y
#         roi = image_hsv[mouseY-10:mouseY+10, mouseX-10:mouseX+10]
#         detect = "Red"
#     elif event == cv.EVENT_RBUTTONDOWN:
#         # cv.circle(new_img, (x, y), 4, (0, 255, 0), 2)
#         mouseX, mouseY = x, y
#         roi = image_hsv[mouseY-10:mouseY+10, mouseX-10:mouseX+10]
#         detect = "Green"

def key_press(event):
    global pwm_input
    key = event.keysym
    if key == 'Up':
        pwm_input = 1
    elif key == 'Down':
        pwm_input = -1

def mouse_click(event):
    global mouseX, mouseY, image_hsv, roi, detect
    if event.num == 1:
        mouseX, mouseY = event.x, event.y
        roi = image_hsv[mouseY-10:mouseY+10, mouseX-10:mouseX+10]
        detect = "Red"
    elif event.num == 3:
        mouseX, mouseY = event.x, event.y
        roi = image_hsv[mouseY-10:mouseY+10, mouseX-10:mouseX+10]
        detect = "Green"

def image_callback(img):
    global ori
    ori_cv = numpy.fromstring(img.data, numpy.uint8)
    ori_cv = cv.imdecode(ori_cv, cv.IMREAD_COLOR)
    ori = ori_cv

def red_mask_callback(img):
    global red_mask
    mask_cv = numpy.fromstring(img.data, numpy.uint8)
    mask_cv = cv.imdecode(mask_cv, cv.IMREAD_COLOR)
    red_mask = mask_cv

def green_mask_callback(img):
    global green_mask
    mask_cv = numpy.fromstring(img.data, numpy.uint8)
    mask_cv = cv.imdecode(mask_cv, cv.IMREAD_COLOR)
    green_mask = mask_cv

def throttle_pwm_callback(pwm):
    global throttle_pwm
    throttle_pwm = pwm.data

def mode_callback(mode_in):
    global mode
    if mode_in.value == Mode.ARMED:
        mode = "Armed"
    elif mode_in.value == Mode.HOLD:
        mode = "Hold"

def auto_control_callback(msg):
    global auto_ctrl
    if msg.state == AutoControl.AVOID_RED:
        auto_ctrl = "Red"
    elif msg.state == AutoControl.AVOID_RED_AND_GREEN:
        auto_ctrl = "Red and Green"
    else:
        auto_ctrl = "None"

if __name__ == '__main__':
    # Initialize gcs node
    rospy.init_node('gcs', anonymous=True)

    # Set Publisher nodes
    cfg_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)
    pwm_input_publisher = rospy.Publisher("/makarax/pwm_input", Int16, queue_size=8)

    # Set Subscriber nodes
    img_subscriber = rospy.Subscriber("/makarax/image/processed/compressed", CompressedImage, image_callback)
    auto_ctrl_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    red_mask_subscriber = rospy.Subscriber("/makarax/image/mask/red/compressed", CompressedImage, red_mask_callback)
    green_mask_subscriber = rospy.Subscriber("/makarax/image/mask/green/compressed", CompressedImage, green_mask_callback)
    pwm_subscriber = rospy.Subscriber("/makarax/pwm_throttle", UInt16, throttle_pwm_callback)
    mode_subscriber = rospy.Subscriber("/makarax/mode", Mode, mode_callback)

    master = Tkinter.Tk()
    master.title("Config")
    
    master_width = master.winfo_screenwidth()
    master_height = master.winfo_screenheight()

    master.geometry(str(master_width) + "x" + str(master_height))
    master.bind("<Key>", key_press)

    motor_info_frame = Tkinter.Frame(master=master)

    pwm_label_title = Tkinter.Label(motor_info_frame, text="PWM:", fg='black', font=("Helvetica", 12))
    pwm_label_title.grid(row=1, column=1, sticky="e")
    pwm_label_value = Tkinter.Label(motor_info_frame, text=str(throttle_pwm), fg='black', font=("Helvetica", 12, 'bold'))
    pwm_label_value.grid(row=1, column=2, sticky="w")

    mode_label_title = Tkinter.Label(motor_info_frame, text="Mode:", fg='black', font=("Helvetica", 12))
    mode_label_title.grid(row=2, column=1, sticky="e")
    mode_label_value = Tkinter.Label(motor_info_frame, text=mode, fg='black', font=("Helvetica", 12, 'bold'))
    mode_label_value.grid(row=2, column=2, sticky="w")

    auto_label_title = Tkinter.Label(motor_info_frame, text="Avoid:", fg='black', font=("Helvetica", 12))
    auto_label_title.grid(row=3, column=1, sticky="e")
    auto_label_value = Tkinter.Label(motor_info_frame, text=auto_ctrl, fg='black', font=("Helvetica", 12, 'bold'))
    auto_label_value.grid(row=3, column=2, sticky="w")

    motor_info_frame.grid(row=1, column=1, pady=(0, 5))

    # Set up slider
    slider_frame = Tkinter.Frame(master=master)

    contrast = add_slider('Contrast', -255, 255, 1, slider_frame, 1, 0)
    brightness = add_slider('Brightness', -127, 127, 1, slider_frame, 2, -2)
    gamma = add_slider('Gamma', 0.1, 3, 0.1, slider_frame, 3, 1)
    roi_y = add_slider('ROI Y', 0, 480, 1, slider_frame, 4, 5)
    adjust = add_slider('Adjust', 0, 200, 1, slider_frame, 5, 50)

    # Set Default Value
    red_low_hue = 118    # add_slider('RED L-HUE', 0, 255, 1, slider_frame, 118)
    red_low_sat = 77     # add_slider('RED L-SAT', 0, 255, 1, slider_frame, 77)
    red_low_val = 0      # add_slider('RED L-VAL', 0, 255, 1, slider_frame, 0)
    red_high_hue = 186   # add_slider('RED H-HUE', 0, 255, 1, slider_frame, 186)
    red_high_sat = 255   # add_slider('RED H-SAT', 0, 255, 1, slider_frame, 255)
    red_high_val = 255   # add_slider('RED H-VAL', 0, 255, 1, slider_frame, 255)

    green_low_hue = 69   # add_slider('GREEN L-HUE', 0, 255, 1, slider_frame2, 69)
    green_low_sat = 43   # add_slider('GREEN L-SAT', 0, 255, 1, slider_frame2, 43)
    green_low_val = 0    # add_slider('GREEN L-VAL', 0, 255, 1, slider_frame2, 0)
    green_high_hue = 99  # add_slider('GREEN H-HUE', 0, 255, 1, slider_frame2, 99)
    green_high_sat = 255 # add_slider('GREEN H-SAT', 0, 255, 1, slider_frame2, 255)
    green_high_val = 255 # add_slider('GREEN H-VAL', 0, 255, 1, slider_frame2, 255)

    slider_frame.grid(row=1, column=3, pady=(0, 5))

    # Output Mask value
    mask_info_frame = Tkinter.Frame(master=master)
    red_hue_label_title = Tkinter.Label(mask_info_frame, text="Red Hue:", fg='black', font=("Helvetica", 12))
    red_hue_label_title.grid(row=1, column=1, sticky="e")
    red_hue_label_value = Tkinter.Label(mask_info_frame, text=str(red_low_hue) + ", " + str(red_high_hue), fg='red', font=("Helvetica", 12))
    red_hue_label_value.grid(row=1, column=2, sticky="w")

    red_sat_label_title = Tkinter.Label(mask_info_frame, text="Red Sat:", fg='black', font=("Helvetica", 12))
    red_sat_label_title.grid(row=2, column=1, sticky="e")
    red_sat_label_value = Tkinter.Label(mask_info_frame, text=str(red_low_sat) + ", " + str(red_high_sat), fg='red', font=("Helvetica", 12))
    red_sat_label_value.grid(row=2, column=2, sticky="w")

    red_val_label_title = Tkinter.Label(mask_info_frame, text="Red Val:", fg='black', font=("Helvetica", 12))
    red_val_label_title.grid(row=3, column=1, sticky="e")
    red_val_label_value = Tkinter.Label(mask_info_frame, text=str(red_low_val) + ", " + str(red_high_val), fg='red', font=("Helvetica", 12))
    red_val_label_value.grid(row=3, column=2, sticky="w")

    green_hue_label_title = Tkinter.Label(mask_info_frame, text="Green Hue:", fg='black', font=("Helvetica", 12))
    green_hue_label_title.grid(row=5, column=1, sticky="e")
    green_hue_label_value = Tkinter.Label(mask_info_frame, text=str(green_low_hue) + ", " + str(green_high_hue), fg='green', font=("Helvetica, 12"))
    green_hue_label_value.grid(row=5, column=2, sticky="w")

    green_sat_label_title = Tkinter.Label(mask_info_frame, text="Green Sat:", fg='black', font=("Helvetica", 12))
    green_sat_label_title.grid(row=6, column=1, sticky="e")
    green_sat_label_value = Tkinter.Label(mask_info_frame, text=str(green_low_sat) + ", " + str(green_high_sat), fg='green', font=("Helvetica", 12))
    green_sat_label_value.grid(row=6, column=2, sticky="w")

    green_val_label_title = Tkinter.Label(mask_info_frame, text="Green Val:", fg='black', font=("Helvetica", 12))
    green_val_label_title.grid(row=7, column=1, sticky="e")
    green_val_label_value = Tkinter.Label(mask_info_frame, text=str(green_low_val) + ", " + str(green_high_val), fg='green', font=("Helvetica", 12))
    green_val_label_value.grid(row=7, column=2, sticky="w")

    mask_info_frame.grid(row=1, column=2, pady=(0, 5))

    ori_label = Tkinter.Label(master=master, image=None)
    ori_label.bind("<Button>", mouse_click)
    ori_label.grid(row=2, column=1, columnspan=2)

    red_mask_label = Tkinter.Label(master=master, image=None)
    red_mask_label.grid(row=2, column=3)

    green_mask_label = Tkinter.Label(master=master, image=None)
    green_mask_label.grid(row=2, column=4)

    master.update()

    # Adjust Image
    master_height = master.winfo_height()
    master_width = master.winfo_width()

    photo_height = int(round(master_height - slider_frame.winfo_height()))
    photo_width = int(round(master_width / 3))

    # Check if images' height are larger than the window's available height
    if (int(round(photo_width * 482 / 642) * 3) > photo_height):
        # Calculate image size based on the window's width
        photo_height = int(round(photo_width * 482 / 642))
    else:
        # Calculate image size based on the window's height
        photo_width = int(round(photo_height * 642 / 482))

    # cv.namedWindow("Camera Output")
    # cv.setMouseCallback("Camera Output", selectROI)

    while not rospy.is_shutdown():
        if ori is not None:
            ori_resize = cv.resize(ori, (photo_width, photo_height))
            image_hsv = cv.cvtColor(ori_resize, cv.COLOR_BGR2HSV)
            # cv.imshow("Camera Output", ori)
            b, g, r = cv.split(ori_resize)
            ori_img_array = cv.merge((r, g, b))
            ori_img = Image.fromarray(ori_img_array)
            ori_img_resize = ori_img.resize((photo_width, photo_height), Image.ANTIALIAS)
            ori_img_tk = ImageTk.PhotoImage(image=ori_img_resize)
            ori_label.config(image=ori_img_tk)

        if red_mask is not None:
            b, g, r = cv.split(red_mask)
            red_img_array = cv.merge((r, g, b))
            red_img = Image.fromarray(red_img_array)
            red_img_resize = red_img.resize((photo_width, photo_height), Image.ANTIALIAS)
            red_img_tk = ImageTk.PhotoImage(image=red_img_resize)
            red_mask_label.config(image=red_img_tk)

        if green_mask is not None:
            b, g, r = cv.split(green_mask)
            green_img_array = cv.merge((r, g, b))
            green_img = Image.fromarray(green_img_array)
            green_img_resize = green_img.resize((photo_width, photo_height), Image.ANTIALIAS)
            green_img_tk = ImageTk.PhotoImage(image=green_img_resize)
            green_mask_label.config(image=green_img_tk)

        if mouseX is not None:
            hue = numpy.mean(roi[:,:, 0])
            sat = numpy.mean(roi[:,:, 1])
            val = numpy.mean(roi[:,:, 2])
            hue_low = hue - adjust.get()
            hue_high = hue + adjust.get()
            sat_low = sat - adjust.get()
            sat_high = sat + adjust.get()
            val_low = val - adjust.get()
            val_high = val + adjust.get()

            if detect == "Red":
                red_low_hue = hue_low
                red_low_sat = sat_low
                red_low_val = val_low
                red_high_hue = hue_high
                red_high_sat = sat_high
                red_high_val = val_high
                detect = "None"
            elif detect == "Green":
                green_low_hue = hue_low
                green_low_sat = sat_low
                green_low_val = val_low
                green_high_hue = hue_high
                green_high_sat = sat_high
                green_high_val = val_high
                detect = "None"

        pwm_input_publisher.publish(pwm_input)
        if (pwm_input is 1) or (pwm_input is -1): 
            pwm_input = 0
            
        pwm_label_value.config(text=str(throttle_pwm))
        mode_label_value.config(text=mode)
        auto_label_value.config(text=auto_ctrl)

        red_hue_label_value.config(text=str(round(red_low_hue, 2)) + ", " + str(round(red_high_hue, 2)))
        red_sat_label_value.config(text=str(round(red_low_sat, 2)) + ", " + str(round(red_high_sat, 2)))
        red_val_label_value.config(text=str(round(red_low_val, 2)) + ", " + str(round(red_high_val, 2)))

        green_hue_label_value.config(text=str(round(green_low_hue, 2)) + ", " + str(round(green_high_hue, 2)))
        green_sat_label_value.config(text=str(round(green_low_sat, 2)) + ", " + str(round(green_high_sat, 2)))
        green_val_label_value.config(text=str(round(green_low_val, 2)) + ", " + str(round(green_high_val, 2)))

        master.update()

        # cv.waitKey(30)
        cfg = Config()
        
        cfg.red_low_hue = red_low_hue
        cfg.red_low_sat = red_low_sat 
        cfg.red_low_val = red_low_val 
        cfg.red_high_hue = red_high_hue 
        cfg.red_high_sat = red_high_sat 
        cfg.red_high_val = red_high_val 
        cfg.green_low_hue = green_low_hue 
        cfg.green_low_sat = green_low_sat 
        cfg.green_low_val = green_low_val 
        cfg.green_high_hue = green_high_hue 
        cfg.green_high_sat = green_high_sat 
        cfg.green_high_val = green_high_val 
        cfg.brightness = brightness.get()
        cfg.contrast = contrast.get()
        cfg.gamma = gamma.get()
        cfg.roi_y = roi_y.get()

        cfg_publisher.publish(cfg)
