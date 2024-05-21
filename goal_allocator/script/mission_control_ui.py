#!/usr/bin/env python

from PyQt5.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, 
                             QFileDialog, QMessageBox, QTextEdit, QSizePolicy, QLineEdit, 
                             QTabWidget, QFormLayout, QSpinBox, QGroupBox)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt
from utils.allocator import GoalAllocator
from utils.image_segmentation import edgeDetector
import cv2

class stateMachine():
    INIT_UI=1
    LOAD_CONFIG =2
    IMAGE_LOADED = 3
    EXECUITE =4
class MissionControl(QWidget, GoalAllocator):
    def __init__(self):
        super().__init__()
        self.current_state = stateMachine.INIT_UI
        self.image_path = ""
        self.initUI()        

    def initUI(self):
        self.setWindowTitle("Mission Control")
        
        self.tabs = QTabWidget()
        
        self.main_tab = QWidget()
        self.config_tab = QWidget()
        
        self.initMainTab()
        self.initConfigTab()
        
        self.tabs.addTab(self.main_tab, "Main")
        self.tabs.addTab(self.config_tab, "Config")
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.tabs)
        
        self.setLayout(main_layout)

    def initMainTab(self):
        self.image_label = QLabel(self.main_tab)

        self.load_button = QPushButton("Load Image", self.main_tab)
        self.load_button.clicked.connect(lambda: self.load_image("None"))

        self.execute_button = QPushButton("Execute Mission", self.main_tab)
        self.execute_button.clicked.connect(self.execute_mission)

        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.addWidget(self.load_button)
        button_layout.addWidget(self.execute_button)

        self.log_textedit = QTextEdit(self.main_tab)
        self.log_textedit.setReadOnly(True)
        self.log_textedit.setFixedHeight(75)
        self.log_textedit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        textbox_layout = QVBoxLayout()
        textbox_layout.addWidget(self.image_label)
        textbox_layout.addWidget(self.log_textedit)

        main_tab_layout = QVBoxLayout()
        main_tab_layout.addLayout(button_layout)
        main_tab_layout.addLayout(textbox_layout)
        
        self.main_tab.setLayout(main_tab_layout)

    def initConfigTab(self):
        layout = QVBoxLayout()
        
        num_drones_layout = QFormLayout()
        self.num_drones_input = QSpinBox(self.config_tab)
        self.num_drones_input.setMinimum(1)
        self.num_drones_input.setMaximum(100)
        self.num_drones_input.setValue(5)  # Default value
        num_drones_layout.addRow("Number of Drones:", self.num_drones_input)
        
        drone_area_group = QGroupBox("Drone Show Mapping Area")
        drone_area_layout = QFormLayout()
        
        self.min_x_input = QSpinBox(self.config_tab)
        self.min_x_input.setRange(0, 50)
        self.min_x_input.setValue(3)  # Default value
        drone_area_layout.addRow("Min X:", self.min_x_input)

        self.min_y_input = QSpinBox(self.config_tab)
        self.min_y_input.setRange(0, 50)
        self.min_y_input.setValue(0)  # Default value
        drone_area_layout.addRow("Min Y:", self.min_y_input)

        self.max_x_input = QSpinBox(self.config_tab)
        self.max_x_input.setRange(0, 1000)
        self.max_x_input.setValue(50)  # Default value
        drone_area_layout.addRow("Max X:", self.max_x_input)

        self.max_y_input = QSpinBox(self.config_tab)
        self.max_y_input.setRange(0, 1000)
        self.max_y_input.setValue(50)  # Default value
        drone_area_layout.addRow("Max Y:", self.max_y_input)
        
        drone_area_group.setLayout(drone_area_layout)
        
        layout.addLayout(num_drones_layout)
        layout.addWidget(drone_area_group)

        # Add the Set Config button
        self.set_config_button = QPushButton("Set Config", self.config_tab)
        self.set_config_button.clicked.connect(self.set_config)
        layout.addWidget(self.set_config_button)
        
        self.config_tab.setLayout(layout)

    def load_image(self,image_path = str):

        if image_path == "None":
            self.image_path, _ = QFileDialog.getOpenFileName(self, "Open Image File", "", "Image Files (*.png *.jpg *.jpeg *.bmp)")
            self.current_state = stateMachine.LOAD_CONFIG
            self.set_config()


        if self.image_path:
            processed_img, points = edgeDetector.process_image(self.image_path, self.get_drone_number())
            self.map_goal_point([processed_img.shape[0],processed_img.shape[1]], points)
            resized = cv2.resize(processed_img, (480, 480), interpolation=cv2.INTER_AREA)
            
            qimg = QImage(resized.data, resized.shape[1], resized.shape[0], QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(qimg)
            self.image_label.setPixmap(pixmap)
            self.log_message("Image loaded")
            self.current_state = stateMachine.IMAGE_LOADED

        else:
            self.log_message("Image not loaded")

    def execute_mission(self):
        if (self.current_state == stateMachine.IMAGE_LOADED or self.current_state == stateMachine.EXECUITE)    and self.get_current_drone_count() == self.get_drone_number():
            self.current_state = stateMachine.EXECUITE
            self.update_subscribers()
            self.execuite_formation()
            self.log_message("Mission executed")
            
        elif self.current_state < stateMachine.IMAGE_LOADED:
            self.log_message("Load the image before mission execuitions ")
        else :
            self.log_message("Drone count mismatch")



    def set_config(self):
        # Retrieve values from inputs and log the config settings
        num_drones = self.num_drones_input.value()
        min_x = self.min_x_input.value()
        min_y = self.min_y_input.value()
        max_x = self.max_x_input.value()
        max_y = self.max_y_input.value()
            

        # Log the configuration
        self.log_message("Config set! Number of Drones:{} Min bound: {}, {} Max bound: {}, {}".format(num_drones, min_x, min_y, max_x, max_y))
        
        # Update internal state if necessary
        self.set_drone_number(num_drones)
        self.set_min_bound([min_x, min_y])
        self.set_max_bound([max_x, max_y])

        if self.current_state == stateMachine.IMAGE_LOADED:
            self.load_image(self.image_path)


    def show_message(self, title, text, icon):
        msg = QMessageBox()
        msg.setWindowTitle(title)
        msg.setText(text)
        msg.setIcon(icon)
        msg.exec_()

    def log_message(self, message):
        self.log_textedit.append(message)

if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    ex = MissionControl()
    ex.show()
    sys.exit(app.exec_())
