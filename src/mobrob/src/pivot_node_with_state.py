#! /usr/bin/python3
import numpy as np
import cv2
import PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pickle
import time
import rospy
from basic_motors_and_sensors.msg import WheelCommands
from std_msgs.msg import Int32MultiArray, Bool, Float32, Int32
def main():
    #Setup camera capture and resolution
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280);
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,960);

    # What is the size of each marker - length of a side in meters (or any other unit you are working with). Used in call to "estimatePoseSingleMarkers". 
    marker_side_length = 0.061 # meters 

    #comment this out to remove live display (and some other stuff below)
    # plt.figure()
    #Load calibration from pickle files (Python2 format..  you'll have to recalibrate if you use another camera)
    cam_matrix = pickle.load(open("/home/pi/catkin_ws/src/mobrob/src/cam_matrix.p","rb"),encoding='bytes')
    dist_matrix = pickle.load(open("/home/pi/catkin_ws/src/mobrob/src/dist_matrix.p","rb"),encoding='bytes')

    #setup ROS stuff
    rospy.init_node('aruco_node', anonymous=False)
    pub_aruco = rospy.Publisher('/servo_number', Int32MultiArray, queue_size=1)
    pub_visible = rospy.Publisher('/QR_Recognize', Bool, queue_size=1)
    pub_motor_command = rospy.Publisher("/wheel_command",WheelCommands,queue_size=1)
    #Tell opencv which aruco tags we're using (this should match the generation script)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    state = 0
    def callback(msg_in):
        if msg_in.data==1:
            nonlocal state
            state=1
        else:
            state=0
    state_machine = rospy.Subscriber("/state",Int32,callback)
    while True:
        if state==1:
            #Read a frame from the camera
            retval, frame = camera.read()
            if not retval:
                continue
            #convert to grayscale
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            #try to find fiducials in the image
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
            if ids is not None:
                print("Deteted ids:")
                for id in ids:
                    print("\t{}".format(id))
                pub_aruco.publish(Int32MultiArray(data=ids[0]))
                pub_visible.publish(Bool(True))
                wheel_command_msg = WheelCommands(left = 0, right=0)
                pub_motor_command.publish(wheel_command_msg)
            else:
                wheel_command_msg = WheelCommands(left = -50, right=50)
                pub_motor_command.publish(wheel_command_msg)
            #
                pub_visible.publish(Bool(False))
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:

        pass
