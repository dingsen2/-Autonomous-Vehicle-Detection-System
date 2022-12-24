#!/usr/bin/env python3

from __future__ import print_function
import math
import os
import time
import cv2

import rospy
import imutils
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PERSON_THRESHOLD = 0.1

class SpeedController:
    def __init__(self, target_speed, p=0.4,i=0.01, d=0,I_threshold=0.6, acc_threshold=0.8) -> None:
        self.p = p
        self.i = i
        self.d = d
        self.I_threshold = I_threshold

        self.Pterm = 0
        self.Iterm = 0
        self.Dterm = 0

        self.prev_error = 0
        self.prev_time = time.time()
        self.target_speed = target_speed
        self.acc_threshold = acc_threshold

    def control(self, curr_speed) -> None:
        brake_cmd_f64_cmd = 0.0
        accel_cmd_f64_cmd = 0.0

        curr_time = time.time()
        delta_time = curr_time - self.prev_time

        current_error = self.target_speed - curr_speed

        delta_error = current_error - self.prev_error

        error_deriv = delta_error / delta_time

        self.Pterm = current_error
        self.Dterm = error_deriv
        self.Iterm += current_error * delta_time

        self.Iterm = max(min(self.Iterm, self.I_threshold), -self.I_threshold)

        self.prev_time = curr_time
        self.prev_error = current_error

        ans = self.p * self.Pterm + self.i * self.Iterm + self.d * self.Dterm

        # TODO: figure out const
        error_threshold = 8

        # if too fast
        if current_error < -error_threshold:
            brake_cmd_f64_cmd = 0.75
            # print("Control: Brake")

        # else:
        #     brake_cmd.f64_cmd = 0.0
        #     print("Accelerate")

        if ans > self.acc_threshold:
            ans = self.acc_threshold
        elif ans < 0.2:
            ans = 0.2

        # if not brake
        if current_error >= -error_threshold:
            accel_cmd_f64_cmd = ans
        else:
            accel_cmd_f64_cmd = 0

        # print("Control: acc magnitude ", accel_cmd_f64_cmd)
        # print("Control: brake magnitude ", brake_cmd_f64_cmd)

        return accel_cmd_f64_cmd, brake_cmd_f64_cmd

class PedestrianBrake():
    def __init__(self):

        # Subscriber
        self.enable_subscriber = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size = 1)
        self.accel_subscriber = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size = 1)
        self.brake_subscriber = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size = 1)


        self.enable_publisher = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
        self.accel_publisher = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
        self.brake_publisher = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
        self.steer_publisher = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
        self.gear_publisher = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)

        # Initial Commands
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = 0.0

        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = 0.0

        self.steer_cmd = PositionWithSpeed()

        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2

        self.enable_publisher.publish(Bool(True))
        self.gear_publisher.publish(self.gear_cmd)
        self.brake_publisher.publish(self.brake_cmd)

        # Time
        self.detecting_person = True
        self.last_person = time.time()
        self.person_detected = False

        self.detecting_stop_sign = 2
        self.last_stop_sign = time.time()

        # PID
        self.controller = SpeedController(1.5)
        self.speed_subscriber = rospy.Subscriber( "/pacmod/as_tx/vehicle_speed", Float64, self.speeding)
        

        # Vision
        self.cv_bridge = CvBridge()

        # Pedestrian Detector
        self.detector = cv2.HOGDescriptor()
        self.detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Stop Sign Detector
        self.stop_data = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/stop_sign_classifier_2.xml')

        self.camera_subscriber = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_gray", Image, self.detecting)

    def is_person(self, frame):
        frame = imutils.resize(frame, width=min(400, frame.shape[1]))
        cv2.imwrite('frame.jpg', frame)

        (boxes, weights) = self.detector.detectMultiScale(frame, winStride=(4, 4), padding=(4, 4), scale = 1.05)
        for w in weights:
            if w > PERSON_THRESHOLD:
                return True
        return False

    def is_stop_sign(self, frame):
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        found = self.stop_data.detectMultiScale(img_gray, scaleFactor=1.05, minNeighbors=5, minSize=(5, 5))
        amount_found = len(found)
        # cv2.imwrite('stop.jpg', frame)
        if amount_found != 0:
            # might want to print the image
            for (x, y, width, height) in found:
                cv2.rectangle(img_gray, (x, y), (x + height, y + width), (0, 255, 0), 5)
            cv2.imwrite(f'stop.jpg', img_gray)
            return True
        return False

    # Callback for Detecting
    def detecting(self, frame):
        frame_cv2 = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
        person_existed = self.is_person(frame_cv2)
        stop_sign_existed = self.is_stop_sign(frame_cv2)
        

        if person_existed:
            self.person_detected = True
        else:
            self.person_detected = False
        
        # print("Detecting: is_person: ", person_existed)
        # print("Detecting: is_stop_sign: ", stop_sign_existed)

        curr_t = time.time()
        # print("time diff:", curr_t, self.last_stop_sign, curr_t - self.last_stop_sign)

        if self.detecting_person == False and curr_t - self.last_person > 3.0:
            self.detecting_person = True
            # rospy.loginfo("Restart Detecting Human!")

        if curr_t - self.last_stop_sign > 5 and self.detecting_stop_sign > 0:
            print("=======================================================================================================")
            if stop_sign_existed:
                rospy.loginfo("Stop Sign Detected and Braked!")
                self.detecting_stop_sign = self.detecting_stop_sign - 1
                self.last_stop_sign = time.time()

                self.enable_subscriber.publish(True)
                self.brake_subscriber.publish(f64_cmd = 1.0, enable = True)
            else:
                rospy.loginfo("stop sign not detected!!!!!")
            print("is detecting true", self.detecting_stop_sign)
            # self.detecting_stop_sign = True
            # rospy.loginfo("Restart Detecting Stop Sign!")
            self.last_stop_sign = curr_t

        if person_existed and self.detecting_person:
            rospy.loginfo("Human Detected and Braked!")
            self.detecting_person = False
            self.last_person = time.time()

            self.enable_subscriber.publish(True)
            self.brake_subscriber.publish(f64_cmd = 1.0, enable = True)
            # time.sleep(1)
            # self.brake_subscriber.publish(f64_cmd = 0.0, enable = True)
            #rospy.loginfo("Brake Stopped!")

        # elif stop_sign_existed and self.detecting_stop_sign and curr_t - self.last_stop_sign > 3:
        #     rospy.loginfo("Stop Sign Detected and Braked!")
        #     self.detecting_stop_sign == False
        #     self.last_stop_sign = time.time()

        #     self.enable_subscriber.publish(True)
        #     self.brake_subscriber.publish(f64_cmd = 1.0, enable = True)
            # time.sleep(1)
            # self.brake_subscriber.publish(f64_cmd = 0.0, enable = True)
            #rospy.loginfo("Brake Stopped!")

        # else:
            # rospy.loginfo("Keep Detecting!")
    
    # Callback for Controlling
    def speeding(self, data):
        # print("speeding: Current data: " + str(data))
        # print("speeding: Current data.data: " + str(data.data))

        curr_t = time.time()
        if curr_t - self.last_person > 4.0 and curr_t - self.last_stop_sign > 2.0 and self.person_detected == False:
            # rospy.loginfo("Controlling the Vehicle!")

            self.accel_cmd.enable = True
            self.accel_cmd.clear = False
            self.accel_cmd.ignore = False

            self.brake_cmd.enable = True
            self.brake_cmd.clear = False
            self.brake_cmd.ignore = False

            accel_cmd_f64_cmd, brake_cmd_f64_cmd = self.controller.control(data.data)

            self.accel_cmd.f64_cmd = accel_cmd_f64_cmd
            self.brake_cmd.f64_cmd = brake_cmd_f64_cmd

            self.accel_publisher.publish(self.accel_cmd)
            self.brake_publisher.publish(self.brake_cmd)

if  __name__ == '__main__':
    rospy.init_node('pedestrian_brake', anonymous = True)
    node = PedestrianBrake()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program Stopped")
