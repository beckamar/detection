import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import Qt 
from interface import Ui_MainWindow  
import json
from imageDetectionManager import ImageDetectionManager



class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("interface.ui", self)

        # Load and apply the custom style from style.json
        self.load_style("style.json")

        # Create image detection manager and pass UI
        self.image_detection_manager = ImageDetectionManager(self.ui)

        self.turnOnCamera.clicked.connect(self.image_detection_manager.start_detection)
        self.turnOffCamera.clicked.connect(self.image_detection_manager.stop_detection)
        
        # Connect custom buttons to their functions
        self.btnMinimize.clicked.connect(self.showMinimized)
        self.btnClose.clicked.connect(self.close)
        self.btnRestore.clicked.connect(self.toggle_maximize_restore)
        
        # Variable to keep track of maximized state
        self.is_maximized = False

        # Connect navigation buttons
        self.homeBtn.clicked.connect(lambda: self.navigate_to("mainSection"))
        self.cameraBtn.clicked.connect(lambda: self.navigate_to("cameraSection"))
        self.sensorsBtn.clicked.connect(lambda: self.navigate_to("sensorSection"))

    def load_style(self, style_path):
        with open(style_path, "r") as file:
            style = json.load(file)
        
        if "QMainWindow" in style:
            window_style = style["QMainWindow"][0]
            if window_style.get("frameless"):
                self.setWindowFlags(Qt.FramelessWindowHint)
            
            if window_style.get("translucentBg"):
                self.setAttribute(Qt.WA_TranslucentBackground)

    def toggle_maximize_restore(self):
        if self.is_maximized:
            self.showNormal()
        else:
            self.showMaximized()
        self.is_maximized = not self.is_maximized

    def navigate_to(self, section_name):
        self.stackedWidgetCenter.setCurrentWidget(getattr(self, section_name))


if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
