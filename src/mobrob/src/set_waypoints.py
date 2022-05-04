#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439WaypointXY
from std_msgs.msg import Bool, Int32

# List of paths supported by the node
waypoints = [
    np.array([[0, 0]]),
    np.array([[0, 1.1], [-0.6, 1.1]]),
    np.array([[0, 1.5], [0, 1.8]]),
    np.array([[0, 0.5],[0.6,0.5]]),
    np.array([[0.5, 0.],[0.5,0.5],[0.,0.5],[0.,0.]]),
]

waypoint_number = 0 # Index of waypoint in path 
path_index = 0
path_complete = Bool()
path_complete.data = True

# Publish desired waypoints at the appropriate time. 
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_waypoint, pub_path_complete
    rospy.init_node('set_waypoints', anonymous=False)
    
    msg_waypoint = ME439WaypointXY()
    
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)
    sub_new_path = rospy.Subscriber('/new_path', Int32, setup_new_path)

    r = rospy.Rate(10)
    try: 
        while not rospy.is_shutdown():
            pub_path_complete.publish(path_complete)
            if not path_complete.data:
                msg_waypoint.x = waypoints[path_index][waypoint_number,0]
                msg_waypoint.y = waypoints[path_index][waypoint_number,1]
                
                pub_waypoint_xy.publish(msg_waypoint)
            
            r.sleep()

    except Exception:
        traceback.print_exc()
        pass
        
        
def setup_new_path(msg_in):
    # FIXME: these globals are awful
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete, path_index
    waypoint_number = 0
    path_index = msg_in.data
    path_complete.data = False

def increment_waypoint(msg_in):
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
    if msg_in.data:
        waypoint_number = waypoint_number + 1
    
    # If the last waypoint was reached, set "path_complete" and publish it
    path_complete.data = (waypoint_number >= waypoints[path_index].shape[0])
    pub_path_complete.publish(path_complete)


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
