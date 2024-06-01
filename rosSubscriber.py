#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ROSImageSubscriber:
    def __init__(self):
        rospy.init_node('imageCamera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        # Se suscribe al tópico ROS que contiene la imagen procesada
        self.image_sub = rospy.Subscriber('lab_image', Image, self.callback)

        #self.detection = Detection()
      #  print("Primer break point")


    def callback(self, data):
        # Convierte la imagen ROS a OpenCV y la envía al objeto Detection para procesamiento
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow("lab camara", cv_image)
            cv2.waitKey(1)
            #self.detection.process_frame(cv_image, new_detection=False)
        except Exception as e:
            print(e)

    def spin_(self):
        # mantener activo el nodo 
        rospy.spin()

if __name__ == '__main__':
    try:
        ros_subscriber = ROSImageSubscriber()
        ros_subscriber.spin_()
    except rospy.ROSInterruptException:
        pass
