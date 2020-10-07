#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication, QMessageBox, QPushButton, QWidget

def onButtonClicked():
    alert = QMessageBox()
    alert.setText('You Clicked the button!')
    alert.exec_()

app = QApplication([])

window = QWidget()
window.setWindowTitle('PyQt5 Button Click')
window.setGeometry(10, 10, 320, 200)

button = QPushButton('Click Me', window)
button.setToolTip('Click Me')
button.clicked.connect(onButtonClicked)
button.show()

app.exec_()