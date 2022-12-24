#!/usr/bin/env python3

from __future__ import print_function
import math
import numpy as np
import os
import time
import cv2

import rospy
import imutils
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from std_msgs.msg import Bool, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva # TurningAround
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
        self.gear_cmd.ui16_cmd = 3

        self.enable_publisher.publish(Bool(True))
        self.gear_publisher.publish(self.gear_cmd)
        self.brake_publisher.publish(self.brake_cmd)

        # TurningAround
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.turning    = 0 
        self.desired_heading    = 0.0
        self.last_time    = time.time()

        # Time
        self.detecting_person = True
        self.last_person = time.time()
        self.person_detected = False

        self.detecting_stop_sign = True
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
        self.stop_data = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/stop_data.xml')

        self.camera_subscriber = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_gray", Image, self.detecting)
    
    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees
        
        print("inspva_callback: Current heading: " + self.heading)
        print("inspva_callback: Current desired heading: " + self.desired_heading)
        if self.turning == -1:
            self.steer_cmd.angular_position = np.radians(-540)
            self.steer_pub.publish(self.steer_cmd)
        
        elif self.turning == 1:
            self.steer_cmd.angular_position = np.radians(540)
            self.steer_pub.publish(self.steer_cmd)
        
        elif abs(self.desired_heading - self.heading) < 20:
            self.turning == 0
            self.steer_cmd.angular_position = np.radians(0)
            self.steer_pub.publish(self.steer_cmd)

    # Callback for Detecting
    def detecting(self, frame):
        # Turn right
        if time.time() - self.last_time > 20 and self.turning == 0:
            self.turning = -1 
            self.desired_heading = (self.heading - 90 + 360) % 360
            self.last_time = time.time()
        
        # Turn left
        # if time.time() - self.last_time > 10 and self.turning == 0:
        #     self.turning = 1
        #     self.desired_heading = (self.heading + 90 + 360) % 360

    
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
