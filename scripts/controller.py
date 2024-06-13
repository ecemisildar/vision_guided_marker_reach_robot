#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
from assignment1_EXP.msg import Marker
import time

IMG_WIDTH = 800
IMG_HEIGHT = 800
EXTRA_RANGE = 20
LIMIT_DISTANCE = 200

# Velocity for real robot: 0.1

class RobotController():
    def __init__(self):
        self.state = 0
        self.center_point = Point()
        self.ids = [11,12,13,15] # Change here the goal markers
        self.current_goal_idx = 0
        self.current_marker = 0
        self.marker_width = 0
        self.vel_msg = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/marker_pub', Marker, self.cb_marker)


    # Callback of the subscription to the topic to get the custom message with information from the marker
    def cb_marker(self, msg):
        self.current_marker = msg.id
        self.center_point.x = ((msg.corners[0].x - msg.corners[1].x)/2)+msg.corners[1].x
        self.center_point.y = ((msg.corners[1].y - msg.corners[2].y)/2)+msg.corners[2].y
        self.marker_width = msg.corners[0].x - msg.corners[1].x


    # Rotation state function that keeps rotating with a constant velocity
    def rotation_state(self):
        print("Current goal:", self.ids[self.current_goal_idx], " marker currently seeing:", self.current_marker)

        if(self.current_marker == self.ids[self.current_goal_idx]):
            print("Found!")
            self.state = 1
            self.vel_msg.angular.z = 0

        else:
            self.vel_msg.angular.z = 2


    # State to center the marker in the middle once seen in the camera, the velocities are changing depending to the distance
    def centering_state(self):
        if(self.center_point.x < (IMG_WIDTH/2-EXTRA_RANGE)):
            # Rotate to the left
            vel = -0.004*self.center_point.x+2.2
            self.vel_msg.angular.z = vel

        elif(self.center_point.x > (IMG_WIDTH/2+EXTRA_RANGE)):
            # Rotate to the right
            vel = -0.00334*self.center_point.x-0.5
            self.vel_msg.angular.z = -vel

        elif(self.center_point.x > (IMG_WIDTH/2-EXTRA_RANGE) and self.center_point.x < (IMG_WIDTH/2+EXTRA_RANGE)):
            print("Centered!")
            self.state = 2
            self.vel_msg.angular.z = 0


    # Function to keep going forward until the marker is close enough. The velocity is implemented proportionally to the distance to the marker
    def forward_state(self):
        if(abs(self.marker_width) >= LIMIT_DISTANCE):
            print("Close enough!")
            if(self.current_goal_idx+1 == len(self.ids)):
                self.state = 3
            else:
                self.current_goal_idx += 1
                self.state = 0
            self.vel_msg.linear.x = 0

        else:
            vel = -0.00699*self.marker_width+1.59
            if(vel > 1):
                vel = 1
            if(vel < 0):
                vel = 0
            self.vel_msg.linear.x = vel


    def planning(self):
        self.state = 0
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.vel_msg.linear.x = 0
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = 0
            
            if self.state == 0:
                print("ROTATING... ")
                self.rotation_state()

            elif self.state == 1:
                print("CENTERING... ", self.center_point)
                self.centering_state()

            elif self.state == 2:
                self.forward_state()
                print("GOING FORWARD...", self.marker_width)

            elif self.state == 3:
                print("FINISHED")

            self.pub.publish(self.vel_msg)

            rate.sleep()

    
if __name__ == "__main__":
    time.sleep(2)

    rospy.init_node('Controller_node')

    robot_controller = RobotController()
 
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        robot_controller.planning()
        rate.sleep()
