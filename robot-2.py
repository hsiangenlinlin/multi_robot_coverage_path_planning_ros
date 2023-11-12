#! /usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/robot2.py

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt,pi
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from beginner_msgsrv.msg import NavData

class Robot:

    STOP = 0
    MOVING_STRAIGHT = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    MOVE_R = 4
    MOVE_TO_GOAL = 5
    FINISHED=6

    SPEED = 0.8

    def __init__(self):
        
        rospy.init_node('robot_2', anonymous=True)
        # get R from yaml
        self.R = rospy.get_param("r")
        self.pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/robot_1/base_scan', LaserScan, self.callback)

        self.min_distance = 1.5

        # set initial state as moving_straight
        self.state = self.MOVING_STRAIGHT
        
        self.turn_time = rospy.Time.now()
        self.move_r_time = rospy.Time.now()

        self.turn_duration = pi / 2.015/ 0.5
        self.move_r_duration = self.R / self.SPEED

        self.count=1
        self.left_count=0

        # Subscribe to Robot-1's position topic
        self.odom_sub=rospy.Subscriber('/robot0/base_pose_ground_truth', Odometry, self.update_odom)

        self.goal_x=0.0
        self.goal_y=15.0

        self.vel_subscriber = rospy.Subscriber('/robot_1/odom', Odometry, self.update_pose)
        self.odom = Odometry()
        self.pose = self.odom.pose.pose
        self.rate = rospy.Rate(10)
        self.roll = self.pitch = self.yaw = 0

        self.visited_cells = set()
	self.cell_number = 0
        self.last_cell = 0
	
	# Subscribe to navigation data from robot-1
	self.nav_sub = rospy.Subscriber('navigation_data_robot_0', NavData, self.nav_callback)
	self.cell_number_from_another=0
        self.last_cell_from_another=0
	

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
            if rospy.Time.now() < self.move_r_time + rospy.Duration(self.move_r_duration):
                vel_msg.linear.x = self.SPEED
            else:
                vel_msg.linear.x = 0.0
                self.count+=1
                self.state = self.get_next_state()
                self.turn_time = rospy.Time.now()

	elif self.state == self.MOVE_TO_GOAL:
                if rospy.Time.now() < self.turn_time + rospy.Duration(self.turn_duration):
                    vel_msg.angular.z = 0.5
                else:
                    vel_msg.angular.z = 0.0
                    self.move_r_time = rospy.Time.now()
                    # move to initial position of robot-1
                    self.move2goal()
                    self.state = self.STOP

        elif self.state == self.STOP:
		vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                # publish cell_number and last_cell once the movements are done
	        self.publish_navigation_data(self.cell_number, self.last_cell)
                    
        self.pub.publish(vel_msg)

    def nav_callback(self, data):
	   self.cell_number_from_another = data.cellNumber
           self.last_cell_from_another = data.lastCell
	   print('data from robot_0, ', 'cell_number: ', data.cellNumber, 'last_cell: ',data.lastCell)

    def get_next_state(self):
        if (self.count-1)%4 == 0:
            return self.MOVING_STRAIGHT
        elif(self.count-3)%4 == 0:
            return self.MOVE_R
        elif self.count%8 in [2,4]:
            if self.left_count<2:
                return self.TURN_LEFT
            else:
                return self.MOVE_TO_GOAL
            
        elif self.count%8 in [6,0]:
            return self.TURN_RIGHT

    #get goal position from robot-0
    def update_odom(self, data):
        self.goal_x = data.pose.pose.position.x
        self.goal_y = data.pose.pose.position.y

   
    def update_pose(self, data):
	# for move2goal()
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
	
	# for getting current cell id
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
    
        current_cell = self.get_cell_id(x, y)

        self.visited_cells.add(current_cell)  # Add current cell to visited cells set

        self.cell_number = len(self.visited_cells)  # Calculate the number of unique visited cells
        self.last_cell = current_cell  # Update the last cell ID

        # Publish navigation data
        self.publish_navigation_data(self.cell_number, self.last_cell)

    def publish_navigation_data(self, cell_number, last_cell):
        #print('cell_number: ',cell_number,'last_cell:', last_cell)
        navigation_pub = rospy.Publisher('navigation_data_robot_2', NavData, queue_size=10)
        navigation_data = NavData()
        navigation_data.cellNumber = cell_number
        navigation_data.lastCell = last_cell
        navigation_pub.publish(navigation_data)

    

    #return distance btw goal_pose and current position
    def euclidean_distance(self, goal_pose):
       return sqrt(pow((goal_pose.position.x-self.pose.position.x), 2)+pow((goal_pose.position.y-self.pose.position.y), 2))
    

    def linear_vel(self, goal_pose, constant=0.3):
        return constant * self.euclidean_distance(goal_pose)
    
    #return the angel turtle should turn
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y-self.pose.position.y,goal_pose.position.x-self.pose.position.x)

    #return angular velocity
    def angular_vel(self, goal_pose, constant=0.5):
        return constant * (self.steering_angle(goal_pose)-self.yaw)

    def move2goal(self):
    
        goal_pose = self.odom.pose.pose
        
        goal_pose.position.x= self.goal_x
        goal_pose.position.y= self.goal_y
        
        dist_tolerance = 0.04
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= dist_tolerance:
            
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.pub.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
     
        self.pub.publish(vel_msg)
    
    def get_cell_id(self, x,y):
        a=int((x+1.5)/3)
        b=int((y+1.5)/3)
	# return cell id
        return (a+1)*6-b


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        robot2 = Robot()
        robot2.run()
    except rospy.ROSInterruptException:
        pass
