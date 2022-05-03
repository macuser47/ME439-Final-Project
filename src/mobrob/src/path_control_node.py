#!/usr/bin/env python3
import rospy
from gyro import Gyro
from std_msgs.msg import Float32
import threading
import time


if __name__ == "__main__":
    rospy.init_node("path_control", anonymous=False)

    path_pub = rospy.Publisher("/new_path", Int32, queue_size=1)

    last = -1
    def on_delivery(msg):
        delivery_ct = msg
        if delivery_ct != last and delivery_ct >= 0:
            path_pub.publish(Int32(delivery_ct))
        last = delivery_ct

    # Set up listener for delivery complete event to queue next path
    del_sub = rospy.Subscriber("/deliveries", Int32, on_delivery)

    rospy.spin()
