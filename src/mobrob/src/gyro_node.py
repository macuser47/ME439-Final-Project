#!/usr/bin/env python3
import rospy
from gyro import Gyro
from std_msgs.msg import Float32
import threading
import time

class StateContainer:
    def __init__(self):
        self.angvel = 0
        self.angle = 0

def read_gyro(s):
    g = Gyro()
    last = time.time()
    now = time.time()
    while True:
        s.angvel = g.angular_velocity()
        now = time.time()
        dt = now - last
        s.angle += s.angvel * dt

        last = now
        

if __name__ == "__main__":
    rospy.init_node("gyro_read", anonymous=False)
    # Set up publishers
    angvel_pub = rospy.Publisher("/gyro_angvel", Float32, queue_size=1)
    angle_pub = rospy.Publisher("/gyro_angle", Float32, queue_size=1)

    # Set up integrator thread 
    s = StateContainer()
    threading.Thread(target=read_gyro, args=(s,),daemon=True).start()

    # Spin and publish
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        angvel_pub.publish(Float32(s.angvel))
        angle_pub.publish(Float32(s.angle))
        r.sleep()

