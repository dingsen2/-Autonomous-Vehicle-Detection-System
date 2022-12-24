#!/usr/bin/env python3

#from steer_pid_controller import steer_pid_controller
import rospy
from std_msgs.msg import Bool, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

import time

steer_publisher = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
gear_publisher = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
enable_publisher = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
accel_publisher = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
brake_publisher = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)


enabled = True

# set initial commands
accel_cmd = PacmodCmd()
accel_cmd.enable = True
accel_cmd.clear = False
accel_cmd.ignore = False
accel_cmd.f64_cmd = 0.0

brake_cmd = PacmodCmd()
brake_cmd.enable = True
brake_cmd.clear = False
brake_cmd.ignore = False
brake_cmd.f64_cmd = 0.0

steer_cmd = PositionWithSpeed()

gear_cmd = PacmodCmd()
gear_cmd.ui16_cmd = 3

# init ros node and send initial commands
rospy.init_node("track_stop_sign", anonymous=True)
enable_publisher.publish(Bool(enabled))
gear_publisher.publish(gear_cmd)
brake_publisher.publish(brake_cmd)

# control speed using pid controller
#TODO: figure out I_threshold, p, i, d
class Speed_controller:
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

        global accel_cmd
        global brake_cmd

        global accel_publisher
        global brake_publisher
        global gear_publisher

    def control(self, curr_speed) -> None:
        global accel_cmd
        global brake_cmd

        global accel_publisher
        global brake_publisher
        global gear_publisher

        accel_cmd.enable = True
        accel_cmd.clear = False
        accel_cmd.ignore = False

        brake_cmd.enable = True
        brake_cmd.clear = False
        brake_cmd.ignore = False

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
            brake_cmd.f64_cmd = 0.75
            print("Brake")

        # else:
        #     brake_cmd.f64_cmd = 0.0
        #     print("Accelerate")

        if ans > self.acc_threshold:
            ans = self.acc_threshold
        elif ans < 0.2:
            ans = 0.2

        # if not brake
        if current_error >= -error_threshold:
            accel_cmd.f64_cmd = ans
        else:
            accel_cmd.f64_cmd = 0

        print("acc magnitude: ", accel_cmd.f64_cmd)
        print("brake magnitude: ", brake_cmd.f64_cmd)

        accel_publisher.publish(accel_cmd)
        brake_publisher.publish(brake_cmd)
print("start controler")
controller = Speed_controller(3)

global_time = time.time()
print("end")
def callback(data):

    global global_time
    print("speed: data: " + str(data))
    print("speed: data.data: " + str(data.data))
    curr_t = time.time()
    if curr_t - global_time > 0.3:
        controller.control(data.data)
        global_time = curr_t


print("start subscriber")
speed_subscriber = rospy.Subscriber( "/pacmod/as_tx/vehicle_speed", Float64, callback)
print("end subscriber")


try:
    rospy.spin()
except KeyboardInterrupt:
    print("shut")
