#!/usr/bin/env python
import Tkinter
from PIL import Image, ImageTk

if __name__ == '__main__':
    master = Tkinter.Tk()
    master.title("Config")

    amv_image = Image.open("amv.png")
    amv_image_resize = amv_image.resize((500,500), Image.ANTIALIAS)
    amv_image_tkinter = ImageTk.PhotoImage(image=amv_image_resize)

    amv_label = Tkinter.Label(master=master, image=amv_image_tkinter)
    amv_label.grid(row=1, column=1)

    while True:
        master.update()