import cv2
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from detection import Detection  

class ImageDetectionManager:
    def __init__(self, ui):
        self.detection = Detection()  # Ajusta la importación de la clase Detection 
        self.detection.ImageUpdate.connect(self.update_image)
        self.ui = ui

    def start_detection(self):
        self.detection.start()

    def stop_detection(self):
        self.detection.stop()

    def update_image(self, image):
        qt_image = self.convert_cv_qt(image)
        self.ui.cameraFrameLabel.setPixmap(qt_image)

    def convert_cv_qt(self, cv_img):
        """Convert from an OpenCV image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
