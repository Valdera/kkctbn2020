#!/usr/bin/env python
import Tkinter
from PIL import Image, ImageTk

def add_slider(text, from_, to_, resolution, frame, row, default=0):
    label = Tkinter.Label(frame, text=text, fg='black', font=("Helvetica", 12))
    label.grid(row=row, column=0, padx=5, sticky='se', pady=2)
    scale = Tkinter.Scale(frame, bd=2, from_=from_, to=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, length=300, showvalue=0)
    scale.set(default)
    scale.grid(row=row, column=2, padx=5, sticky='n', pady=0)
    return scale

def add_label(text, frame, row):
    label = Tkinter.Label(frame, text=text, fg='black', font=("Helvetica", 12))
    label.grid(row=row, column=1)
    return label

if __name__ == '__main__':
    master = Tkinter.Tk()
    master.title("Config")

    amv_image = Image.open("amv.png")
    amv_image_resize = amv_image.resize((100,100), Image.ANTIALIAS)
    amv_image_tkinter = ImageTk.PhotoImage(image=amv_image_resize)

    amv_label = Tkinter.Label(master=master, image=amv_image_tkinter)
    amv_label.grid(row=1, column=1, columnspan=3)

    test_slider = add_slider("Test", 0.0, 100.0, 1, master, 2, 50)
    test_slider_value = add_label(str(test_slider.get()), master, 2)

    while True:
        test_slider_value.config(text=str(test_slider.get()))
        master.update()