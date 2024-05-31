import sys
import cv2
from PyQt5 import uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication
from interface import Ui_MainWindow  
from detection import Detection
import json
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("interface.ui", self)

        # Load and apply the custom style from style.json
        self.load_style("style.json")

        self.detection = Detection()
        self.detection.ImageUpdate.connect(self.update_image)

        self.turnOnCamera.clicked.connect(self.start_detection)
        self.turnOffCamera.clicked.connect(self.stop_detection)
        
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

    def start_detection(self):
        self.detection.start()

    def stop_detection(self):
        self.detection.stop()

    def update_image(self, image):
        qt_image = self.convert_cv_qt(image)
        self.cameraFrameLabel.setPixmap(qt_image)

    def convert_cv_qt(self, cv_img):
        """Convert from an OpenCV image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

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
