#! /usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/robot0.py

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from beginner_msgsrv.msg import NavData

class Robot:

    STOP = 0
    MOVING_STRAIGHT = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    MOVE_R = 4

    SPEED = 0.7

    def __init__(self):
        rospy.init_node('robot_1', anonymous=True)
        # get R from yaml
        self.R = rospy.get_param("r")
        self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, self.callback)

	# set distance to the wall as 1.5
        self.min_distance = 1.5

        # set initial state as moving ahead
        self.state = self.MOVING_STRAIGHT
        
        self.turn_time = rospy.Time.now()
        self.move_r_time = rospy.Time.now()

        self.turn_duration = math.pi / 2.015/ 0.5
        self.move_r_duration = self.R / self.SPEED
	
	# this is for counting the steps
        self.count=1
        self.left_count=0

        # publish initial position of robot-1 to robot-2 
        self.odom_pub = rospy.Publisher('/robot0/base_pose_ground_truth', Odometry, queue_size=10)
	self.odom_msg = Odometry()
	self.odom_msg.pose.pose.position.x = 0.0
	self.odom_msg.pose.pose.position.y = 15.0
	self.odom_msg.pose.pose.position.z = 0.0
	self.odom_pub.publish(self.odom_msg)
	
    	self.vel_subscriber = rospy.Subscriber('/robot_0/odom', Odometry, self.update_pose)
        self.visited_cells = set()
	self.cell_number = 0
        self.last_cell = 0
        self.cell_number_from_another=0
        self.last_cell_from_another=0

	# Subscribe to navigation data from robot-2
	self.nav_sub = rospy.Subscriber('navigation_data_robot_2', NavData, self.nav_callback)
	
    def nav_callback(self, data):
        self.cell_number_from_another = data.cellNumber
        self.last_cell_from_another = data.lastCell
	   #print('data from robot_2, ', 'cell_number: ', data.cellNumber, 'last_cell: ',data.lastCell)

    def update_pose(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # print(x,y)
        current_cell = self.get_cell_id(x, y)

        self.visited_cells.add(current_cell)  # Add current cell to visited cells set

        self.cell_number = len(self.visited_cells)  # Calculate the number of unique visited cells
        self.last_cell = current_cell  # Update the last cell ID

    def publish_navigation_data(self, cell_number, last_cell):
        # print('cell_number: ',cell_number,'last_cell:', last_cell)
        navigation_pub = rospy.Publisher('navigation_data_robot_0', NavData, queue_size=10)
        navigation_data = NavData()
        navigation_data.cellNumber = cell_number
        navigation_data.lastCell = last_cell
        navigation_pub.publish(navigation_data)


    def callback(self, data):
        range_to_wall = min(data.ranges)
        vel_msg = Twist()

        if self.state == self.MOVING_STRAIGHT:
            if range_to_wall > self.min_distance:
                vel_msg.linear.x = self.SPEED
            else:
                vel_msg.linear.x = 0.0
            
                self.count+=1
                self.state = self.get_next_state()
                self.turn_time = rospy.Time.now()

        elif self.state == self.TURN_RIGHT:
            if rospy.Time.now() < self.turn_time + rospy.Duration(self.turn_duration):
                vel_msg.angular.z = -0.5
            else:
                vel_msg.angular.z = 0.0
                self.count+=1
                self.state = self.get_next_state()
                self.move_r_time = rospy.Time.now()

        elif self.state == self.TURN_LEFT:
            if rospy.Time.now() < self.turn_time + rospy.Duration(self.turn_duration):
                vel_msg.angular.z = 0.5
            else:
                vel_msg.angular.z = 0.0
                self.count+=1
                self.left_count+=1
                self.state = self.get_next_state()
                self.move_r_time = rospy.Time.now()

        elif self.state == self.MOVE_R:
        	# print(self.R)
            if rospy.Time.now() < self.move_r_time + rospy.Duration(self.move_r_duration):
                vel_msg.linear.x = self.SPEED
            else:
                vel_msg.linear.x = 0.0
                self.count+=1
                self.state = self.get_next_state()
                self.turn_time = rospy.Time.now()

        elif self.state == self.STOP:
               vel_msg.linear.x = 0.0
               vel_msg.angular.z = 0.0
	       self.publish_navigation_data(self.cell_number, self.last_cell)
               print('data from robot_2, ', 'cell_number: ', self.cell_number_from_another, 'last_cell: ',self.last_cell_from_another)


        self.pub.publish(vel_msg)


    def get_next_state(self):
    	if (self.count-1)%4 == 0:
    		return self.MOVING_STRAIGHT
    	elif(self.count-3)%4 == 0:
    		return self.MOVE_R
    	elif self.count%8 in [2,4]:
    		return self.TURN_RIGHT
    	elif self.count%8 in [6,0]:
    		if self.left_count<4:
    			return self.TURN_LEFT
    		else:
    			return self.STOP
    
    def get_cell_id(self,x, y):
        a=int((x+1.5)/3)
        b=int((abs(y)+1.5)/3)
        # return cell id here
        return b+a*6+1
    

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        robot0 = Robot()
        robot0.run()
    except rospy.ROSInterruptException:
        pass
