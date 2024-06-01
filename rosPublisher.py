#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Camera Init Function
def cameraInit():
    # Camera initialization (video0 is the Raspberry Pi Cam connected to the CSI port)
    cameraHandle = cv2.VideoCapture(0)
    # Validate if the camera was successfully open
    if cameraHandle.isOpened():
        print("Camera was successfully opened!")
        return cameraHandle
    cameraHandle.release()
    print("Camera was not found")
    return None

# Convert CV2 Frame to sensor_msgs/Image type
def cv2toImgMsg(cv2Img):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(cv2Img, encoding="bgr8")
    return msg

# Main function
def main():
    camera = cameraInit()   # Camera Handle
    ImgMsg = Image()        # Image msg variable
    # Ros Initialization
    rospy.init_node("camera_node")
    publisher = rospy.Publisher('lab_image',Image, queue_size=10)
    rate = rospy.Rate(10)   # Rate at 10Hz
    # ROS loop
    while not rospy.is_shutdown():
        ret, frame = camera.read()  # Image capture
        ImgMsg = cv2toImgMsg(frame)   # Converts to Image msg type
        publisher.publish(ImgMsg)   # Publish the image in ROS
        rate.sleep()                # Sleep for the derired loop rate
    # After loop, it is necessary to release the camera handle
    camera.release()


if __name__ == '__main__':
    try:
        main()
    except rospy.RosInitException:
        pass

