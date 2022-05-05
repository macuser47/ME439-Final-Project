#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import traceback
import Adafruit_PCA9685      ## Alternative library, different function: import pi_servo_hat
# IMPORT the messages:
from std_msgs.msg import Int32, Float32, Bool, Int32MultiArray
# Get all the functions from "servo_examples"
import servo_examples as servos

servos.shutdown_servos()    # Give them a clean start.

state=0
deliveries=None

def main():
    global state
    rospy.init_node('SM_node', anonymous=False)


    statemachine_publisher=rospy.Publisher('/state',Int32,queue_size=1)


    def stop_handler(msg_in): #msg_in is a Bool of type /Destination
        global state
        if state==0:
            if msg_in.data == True:
                if deliveries == 0:
                    state = 2
                else:
                    state = 1
            else:
                state = 0
            statemachine_publisher.publish(state)


    def driving_handler(msg_in):
        global state, deliveries
        deliveries = msg_in.data


    def aruco_handler(msg_in):
        global state
        if state == 1 and msg_in.data:
            state= 0
        statemachine_publisher.publish(state)


    delivery_subscriber = rospy.Subscriber('/deliveries', Int32, driving_handler)
    aruco_subscriber = rospy.Subscriber('/QR_Recognize', Bool,aruco_handler)
    destination_subscriber=rospy.Subscriber('/path_complete',Bool,stop_handler)
    r=rospy.Rate(50)
    while not rospy.is_shutdown():
        statemachine_publisher.publish(state)
        r.sleep()


# Startup stuff.
if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
        # initialize_pwm()  # to shut it down.
        servos.shutdown_servos()    # to shut it down

# Shut down here also, just in case.
# initialize_pwm()  # to shut it down.
servos.shutdown_servos()    # to shut it down

