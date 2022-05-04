#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32 
from states import RobotState
import threading
import time

if __name__ == "__main__":
    rospy.init_node("test_fsm_response", anonymous=False)
    # Set up publishers
    state_pub = rospy.Publisher("/state", Int32, queue_size=1)

    state = RobotState.DRIVING
    # Spin and publish
    while not rospy.is_shutdown():
        print("States: {}".format(list(RobotState)))
        print("Current state: {}".format(state))
        state = RobotState(int(input("New State Number:")))
        state_pub.publish(Int32(data=state.value))
