#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/resetposition.py
import rospy
from std_srvs.srv import Empty

def reset_positions():
    rospy.init_node('reset_positions_node')
    rospy.wait_for_service('/reset_positions')
    
    try:
        reset_service = rospy.ServiceProxy('/reset_positions', Empty)
        reset_service()
        print("Positions reset successfully.")
    except rospy.ServiceException as e:
        print("Failed to reset positions: %s" % e)

if __name__ == '__main__':
    reset_positions()

