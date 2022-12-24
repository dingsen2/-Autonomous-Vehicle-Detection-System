#!/usr/bin/env python3

from __future__ import print_function
import math
import os
import time
import cv2

import rospy
import imutils
from pacmod_msgs.msg import PacmodCmd
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PERSON_THRESHOLD = 0.1
stop_data = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/stop_data.xml')

class PedestrianBrake():
    def __init__(self):
        self.enable_subscriber = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size = 1)
        self.accel_subscriber = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size = 1)
        self.brake_subscriber = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size = 1)
        self.cv_bridge = CvBridge()

        self.detector = cv2.HOGDescriptor()
        self.detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        #self.camera_subscriber = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.detecting)
        self.camera_subscriber = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_gray", Image, self.detecting)

    def is_person(self, frame):
        frame = imutils.resize(frame, width=min(400, frame.shape[1]))

        # frame = cv2.resize(frame, (640, 480))
        #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        cv2.imwrite('frame.jpg', frame)

        (boxes, weights) = self.detector.detectMultiScale(frame, winStride=(4, 4), padding=(4, 4), scale = 1.05)

        #rospy.loginfo(frame[0:5][0:5])
        #rospy.loginfo(len(boxes))
        #rospy.loginfo(self.detector.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale = 1.05))

        for w in weights:
            if w > PERSON_THRESHOLD:
                return True
        return False

    def is_stop_sign(self, frame):
        # img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        found = stop_data.detectMultiScale(frame, minSize =(20, 20))
        amount_found = len(found)
        if amount_found != 0:
            # might want to print the image
            return True
        return False


    def detecting(self, frame):
        frame_cv2 = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
        existed = self.is_person(frame_cv2) or self.is_stop_sign(frame_cv2)

        if existed:
            rospy.loginfo("Human Detected!")
            self.enable_subscriber.publish(True)
            self.brake_subscriber.publish(f64_cmd = 1.0, enable = True)
            # time.sleep(1)
            # self.brake_subscriber.publish(f64_cmd = 0.0, enable = True)
            rospy.loginfo("Brake Stopped!")
        else:
            rospy.loginfo("Keep Detecting!")

if  __name__ == '__main__':
    rospy.init_node('pedestrian_brake', anonymous = True)
    node = PedestrianBrake()
    #node.accel_subscriber.publish(f64_cmd = 0.5, enable = True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program Stopped")
