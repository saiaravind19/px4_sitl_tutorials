import sys
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QFileDialog, QMessageBox, QTextEdit, QSizePolicy
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

class MissionControl(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Mission Control")

        self.image_label = QLabel(self)

        self.load_button = QPushButton("Load Image", self)
        self.load_button.clicked.connect(self.load_image)

        self.execute_button = QPushButton("Execute Mission", self)
        self.execute_button.clicked.connect(self.execute_mission)

        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.addWidget(self.load_button)
        button_layout.addWidget(self.execute_button)

        self.log_textedit = QTextEdit(self)
        self.log_textedit.setReadOnly(True)
        
        self.log_textedit.setFixedHeight(75)

        self.log_textedit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  
        textbox_layout = QVBoxLayout()
        textbox_layout.addWidget(self.image_label)
        textbox_layout.addWidget(self.log_textedit)

        main_layout = QVBoxLayout()
        main_layout.setSpacing(0)  
        main_layout.addLayout(button_layout)
        main_layout.addLayout(textbox_layout)

        self.setLayout(main_layout)

    def load_image(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Image File", "", "Image Files (*.png *.jpg *.jpeg *.bmp)")
        if file_path:
            pixmap = QPixmap(file_path)
            resized_pixmap = pixmap.scaled(640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(resized_pixmap)
            
            self.log_message("Image loaded")
        else:
            self.log_message("Image not loaded")


    def execute_mission(self):
        # Your mission execution code goes here
        self.log_message("Mission executed!")

    def show_message(self, title, text, icon):
        msg = QMessageBox()
        msg.setWindowTitle(title)
        msg.setText(text)
        msg.setIcon(icon)
        msg.exec_()

    def log_message(self, message):
        self.log_textedit.append(message)

