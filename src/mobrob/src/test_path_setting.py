#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Int32MultiArray
from states import RobotState
import threading
import time
from contextlib import suppress

def run_test():
    rospy.init_node("test_path_setting", anonymous=False)
    # Set up publishers
    state_pub = rospy.Publisher("/deliveries", Int32, queue_size=1)
    
    dev = 3
    # Spin and publish
    while not rospy.is_shutdown():
        print("Current Deliveries: {}".format(dev))
        dev = int(input("New delivieries number:"))
        state_pub.publish(dev)

if __name__ == "__main__":
    with suppress(EOFError):
        run_test()
