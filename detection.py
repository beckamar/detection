import cv2
import numpy as np
from ultralytics import YOLO
from PyQt5.QtCore import QThread, pyqtSignal



class Detection(QThread):
    ImageUpdate = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.yolo_model = YOLO("best.pt")
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 640)
        self.stopped = False

    def run(self):
        while not self.stopped:
            ret, frame = self.video_capture.read()
            if ret:
                self.process_frame(frame)
        self.video_capture.release()

    def process_frame(self, frame):
        detection_results = self.yolo_model.predict(frame, imgsz=640, conf=0.50)
        annotated_image = detection_results[0].plot()
        self.ImageUpdate.emit(annotated_image)

    def stop(self):
        self.stopped = True
        self.wait()
        cv2.destroyAllWindows()
