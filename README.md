# Autonomous-Vehicle-Detection-System

# Code repo for the vehicle detection system. 
As part of the CS588 course, our team of four (Jiaxi Li, Dingsen Shi, Yijun Lin, and Ang Li) developed an autonomous vehicle system capable of detecting traffic signs and pedestrians in real-time. The system is able to respond appropriately by making quarter turns at turn signs, stopping and waiting at stop signs, and stopping and waiting for pedestrians.

# Implementation:

- We implemented a PID controller to control the acceleration and speed of the vehicle.
- We trained a traffic sign detector based on the Lenet model to detect turn signs in real-time. Whenever a turn sign is detected, we set the target heading and publish the appropriate steer command.
- We used the OpenCV library to construct a model for detecting stop signs in real-time with decent accuracy. Whenever a stop sign is detected, we publish a brake command and pause the acceleration PID controller.
- We used the OpenCV library's HOGDescriptor_getDefaultPeopleDetector to detect pedestrians. Whenever a pedestrian is detected, we publish a brake command and pause the acceleration PID controller until the pedestrian is no longer detected.
