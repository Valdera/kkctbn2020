#!/usr/bin/env python
# If using ROS Noetic on Ubuntu 20.04 change python to python3 to make it working
import cv2 as cv
import numpy
import rospy
import Tkinter
from PIL import Image, ImageTk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16, Int16, Float64, Bool
# Local Import
from kkctbn2020.msg import AutoControl, Config, ObjectCount, Mode

ori = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
red_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)
green_mask = numpy.zeros([480, 640, 3], dtype = numpy.uint8)

throttle_pwm = 0
setpoint = 0
pwm_just_forward = False
auto_ctrl = "Red, Green"
mode = "None"
detect = "None"

red_low_hue, red_low_sat, red_low_val = 118, 77, 0
red_high_hue, red_high_sat, red_high_val = 186, 255, 255

green_low_hue, green_low_sat, green_low_val = 69, 43, 0
green_high_hue, green_high_sat, green_high_val = 99, 255, 255

pwm_input = 0
setpoint_input = 0
frame, threshold, image_hsv = None, None, None
mouseX, mouseY = None, None
roi = None

def nothing(num):
    pass

def add_color(frame, title, value, row, color='black'):
    label_title = Tkinter.Label(frame, text=title, fg='black', font=("Helvetica", 12))
    label_title.grid(row=row, column=1, sticky='e')
    label_value = Tkinter.Label(frame, text=value, fg=color, font=("Helvetica", 12))
    label_value.grid(row=row, column=2, sticky='w')
    return label_value

def add_label(frame, title, value, row):
    label_title = Tkinter.Label(frame, text=title, fg='black', font=("Helvetica", 12))
    label_title.grid(row=row, column=1, sticky='e')
    label_value = Tkinter.Label(frame, text=value, fg='black', font=("Helvetica", 12, 'bold'))
    label_value.grid(row=row, column=2, sticky='w')
    return label_value

def add_slider(frame, from_, to_, resolution, row, default=0):
    scale = Tkinter.Scale(frame, from_=from_, to=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, showvalue=0, length=300)
    scale.set(default)
    scale.grid(row=row, column=3, padx=5, sticky='n', pady=2)
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
    global pwm_input, setpoint_input
    key = event.keysym
    if key == 'Up':
        pwm_input = 1
    elif key == 'Down':
        pwm_input = -1
        
    if key == 'Left':
        setpoint_input = -1
    elif key == 'Right':
        setpoint_input = 1

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

def setpoint_callback(set):
    global setpoint
    setpoint = set.data

def pwm_just_forward_callback(state):
    global pwm_just_forward
    pwm_just_forward = state.data

if __name__ == '__main__':
    # Initialize gcs node
    rospy.init_node('gcs', anonymous=True)

    # Set Publisher nodes
    cfg_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)
    pwm_input_publisher = rospy.Publisher("/makarax/pwm_input", Int16, queue_size=8)
    setpoint_input_publisher = rospy.Publisher("/makarax/setpoint_input", Int16, queue_size=8)
    degree_decision_publisher = rospy.Publisher("/makarax/degree_decision", Int16, queue_size=8)

    # Set Subscriber nodes
    img_subscriber = rospy.Subscriber("/makarax/image/processed/compressed", CompressedImage, image_callback)
    auto_ctrl_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    red_mask_subscriber = rospy.Subscriber("/makarax/image/mask/red/compressed", CompressedImage, red_mask_callback)
    green_mask_subscriber = rospy.Subscriber("/makarax/image/mask/green/compressed", CompressedImage, green_mask_callback)
    pwm_subscriber = rospy.Subscriber("/makarax/pwm_throttle", UInt16, throttle_pwm_callback)
    mode_subscriber = rospy.Subscriber("/makarax/mode", Mode, mode_callback)
    setpoint_subscriber = rospy.Subscriber("setpoint", Float64, setpoint_callback)
    pwm_just_forward_subscriber = rospy.Subscriber("/makarax/pwm_just_forward", Bool, pwm_just_forward_callback)

    master = Tkinter.Tk()
    master.title("Config")
    
    master_width = master.winfo_screenwidth()
    master_height = master.winfo_screenheight()

    master.geometry(str(master_width) + "x" + str(master_height))
    master.bind("<Key>", key_press)

    motor_info_frame = Tkinter.Frame(master=master)

    pwm_label         = add_label(motor_info_frame, "PWM:", str(throttle_pwm), 1)
    mode_label        = add_label(motor_info_frame, "Mode:", mode, 2)
    auto_label        = add_label(motor_info_frame, "Avoid:", auto_ctrl, 3)
    setpoint_label    = add_label(motor_info_frame, "Set:", str(setpoint), 4)
    pwm_forward_label = add_label(motor_info_frame, "State:", str(pwm_just_forward), 5)

    motor_info_frame.grid(row=1, column=2, pady=(5, 5))

    # Set up slider
    slider_frame = Tkinter.Frame(master=master)

    contrast_value = add_slider(slider_frame, -255, 255, 1, 1, 0)
    contrast_label = add_label(slider_frame, 'Contrast:', str(contrast_value.get()), 1)

    brightness_value = add_slider(slider_frame, -127, 127, 1, 2, -2)
    brightness_label = add_label(slider_frame, 'Brightness:', str(brightness_value.get()), 2)

    gamma_value = add_slider(slider_frame, 0.1, 3, 0.1, 3, 1)
    gamma_label = add_label(slider_frame, 'Gamma:', str(gamma_value.get()), 3)

    roi_value = add_slider(slider_frame, 0, 482, 1, 4, 241)
    roi_label = add_label(slider_frame, 'ROI Y:', str(roi_value.get()), 4)

    adjust_value = add_slider(slider_frame, 0, 200, 1, 5, 50)
    adjust_label = add_label(slider_frame, 'Adjust:', str(adjust_value.get()), 5)

    degree_value = add_slider(slider_frame, 0, 180, 1, 6, 95)
    degree_label = add_label(slider_frame, "Degree:", str(degree_value.get()), 6)

    slider_frame.grid(row=1, column=4, pady=(5, 5))

    # Output Mask value
    mask_info_frame = Tkinter.Frame(master=master)

    red_hue_label = add_color(mask_info_frame, "Red Hue:", str(red_low_hue) + ", " + str(red_high_hue), 1, 'red')
    red_sat_label = add_color(mask_info_frame, "Red Sat:", str(red_low_sat) + ", " + str(red_high_sat), 2, 'red')
    red_val_label = add_color(mask_info_frame, "Red Val:", str(red_low_val) + ", " + str(red_high_val), 3, 'red')

    green_hue_label = add_color(mask_info_frame, "Green Hue:", str(green_low_hue) + ", " + str(green_high_hue), 5, 'green')
    green_sat_label = add_color(mask_info_frame, "Green Sat:", str(green_low_sat) + ", " + str(green_high_sat), 6, 'green')
    green_val_label = add_color(mask_info_frame, "Green Sat:", str(green_low_val) + ", " + str(green_high_val), 7, 'green')

    mask_info_frame.grid(row=1, column=3, pady=(5, 5))

    # Image Frame
    ori_label = Tkinter.Label(master=master, image=None)
    ori_label.bind("<Button>", mouse_click)
    ori_label.grid(row=2, column=1)

    red_mask_label = Tkinter.Label(master=master, image=None)
    red_mask_label.grid(row=2, column=2, columnspan=2)

    green_mask_label = Tkinter.Label(master=master, image=None)
    green_mask_label.grid(row=2, column=4)

    master.update()

    # Adjust Image
    master_height = master.winfo_height()
    master_width = master.winfo_width()

    frame_height = max(slider_frame.winfo_height(), motor_info_frame.winfo_height(), mask_info_frame.winfo_height())

    photo_height = int(round(master_height - frame_height))
    photo_width = int(round(master_width / 3))

    # Check if images' height are larger than the window's available height
    if (int(round(photo_width * 482 / 642) * 3) > photo_height):
        # Calculate image size based on the window's width
        photo_height = int(round(photo_width * 482 / 642))
    else:
        # Calculate image size based on the window's height
        photo_width = int(round(photo_height * 642 / 482))

    amv_image = Image.open("amv.png")
    if (frame_height > photo_width):
        frame_height = photo_width
    amv_image_resize = amv_image.resize((frame_height, frame_height), Image.ANTIALIAS)
    amv_image_tkinter = ImageTk.PhotoImage(image=amv_image_resize)

    amv_label = Tkinter.Label(master=master, image=amv_image_tkinter)
    amv_label.grid(row=1, column=1, pady=(5, 5))

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
            ori_img_tk = ImageTk.PhotoImage(image=ori_img)
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
            adjust = adjust_value.get()
            hue = numpy.mean(roi[:,:, 0])
            sat = numpy.mean(roi[:,:, 1])
            val = numpy.mean(roi[:,:, 2])
            hue_low = hue - adjust
            hue_high = hue + adjust
            sat_low = sat - adjust
            sat_high = sat + adjust
            val_low = val - adjust
            val_high = val + adjust

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

        setpoint_input_publisher.publish(setpoint_input)
        if (setpoint_input is 1) or (setpoint_input is -1):
            setpoint_input = 0
            
        pwm_label.config(text=str(throttle_pwm))
        mode_label.config(text=mode)
        auto_label.config(text=auto_ctrl)
        setpoint_label.config(text=str(setpoint))
        pwm_forward_label.config(text=str(pwm_just_forward))

        red_hue_label.config(text=str(round(red_low_hue, 2)) + ", " + str(round(red_high_hue, 2)))
        red_sat_label.config(text=str(round(red_low_sat, 2)) + ", " + str(round(red_high_sat, 2)))
        red_val_label.config(text=str(round(red_low_val, 2)) + ", " + str(round(red_high_val, 2)))

        green_hue_label.config(text=str(round(green_low_hue, 2)) + ", " + str(round(green_high_hue, 2)))
        green_sat_label.config(text=str(round(green_low_sat, 2)) + ", " + str(round(green_high_sat, 2)))
        green_val_label.config(text=str(round(green_low_val, 2)) + ", " + str(round(green_high_val, 2)))
        
        contrast_label.config(text=str(contrast_value.get()))
        brightness_label.config(text=str(brightness_value.get()))
        gamma_label.config(text=str(gamma_value.get()))
        roi_label.config(text=str(roi_value.get()))
        adjust_label.config(text=str(adjust_value.get()))
        degree_label.config(text=str(degree_value.get()))

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
        cfg.brightness = brightness_value.get()
        cfg.contrast = contrast_value.get()
        cfg.gamma = gamma_value.get()
        cfg.roi_y = roi_value.get()

        cfg_publisher.publish(cfg)
        degree_decision_publisher.publish(degree_value.get())
