import cv2
import numpy as np
from ultralytics import YOLO
import torch
import time

class Detection:
    def __init__(self):
        self.yolo_model = YOLO("best.pt")
        self.second_model = YOLO("kind.pt")
        self.video_capture = cv2.VideoCapture("/dev/v4l/by-id/usb-Foxlink_HP_Webcam_0x0001-video-index0")
        time.sleep(1)
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 480)
        self.stopped = False
        self.frame_counter = 0
        self.skip_frames = 5
        self.last_detections = []
        self.detection_lifetime = 4

    def start(self):
        while not self.stopped:
            ret, frame = self.video_capture.read()
            if ret:
                self.frame_counter += 1
                if self.frame_counter % self.skip_frames == 0:
                    self.process_frame(frame, new_detection=True)
                else:
                    self.process_frame(frame, new_detection=False)
        self.video_capture.release()

    def process_frame(self, frame, new_detection=True):
        if new_detection:
            detection_results = self.yolo_model.predict(frame, imgsz=640, conf=0.5)
            self.last_detections = [(detection, self.detection_lifetime) for detection in detection_results]
        else:
            self.last_detections = [(detection, life-1) for detection, life in self.last_detections if life > 0]

        for detection_obj, life in self.last_detections:
            for box in detection_obj.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                label_index = int(box.cls.item())
                label = detection_obj.names[label_index]
                confidence = box.conf.item() if isinstance(box.conf, torch.Tensor) else box.conf
                
                if label.lower() == 'rock':
                    rock_image = frame[int(y1):int(y2), int(x1):int(x2)]
                    rock_type = self.classify_rock_type(rock_image)
                    label = rock_type

                text = f"{label}: {confidence:.2f}"
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                text_x = int(x1)
                text_y = int(y1) - 10 if int(y1) - 10 > 10 else int(y1) + 10
                cv2.rectangle(frame, (text_x, text_y - text_size[1] - 2), (text_x + text_size[0], text_y + 2), (0, 255, 0), -1)
                cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop()

    def classify_rock_type(self, rock_image):
        rgb_image = cv2.cvtColor(rock_image, cv2.COLOR_BGR2RGB)
        rgb_image = convert_to_8bit(rgb_image)
        resized_image = cv2.resize(rgb_image, (640, 640))
        results_list = self.second_model(resized_image)
        if not results_list:
            return "Unknown"

        results = results_list[0]
        rock_type_index = results.probs.top1
        rock_type = results.names[rock_type_index]
        rock_type_confidence = results.probs.top1conf.item() if isinstance(results.probs.top1conf, torch.Tensor) else results.probs.top1conf

        return f"{rock_type}: {rock_type_confidence:.2f}"

    def stop(self):
        self.stopped = True
        cv2.destroyAllWindows()

def convert_to_8bit(image):
    if image.dtype == np.float64:
        image = np.clip(image * 255.0, 0, 255).astype(np.uint8)
    return image
