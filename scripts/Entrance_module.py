#   Libraries for Room Identifier class
from __future__ import division
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#   Libraries for Entrance class
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import yaml
import os
import time

PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')

class RoomIdentifier:                                                                     # Class created to modify Turtlebot's vision to see only green and red colors and utilize that information
    def __init__(self):
        rospy.Subscriber('camera/rgb/image_raw', Image, self.identify_color_callback)     # Set up a subscriber to the image topic
        self.gflag = False                                                                # Initialise colour flags
        self.rflag = False
        self.sensitivity = 5                                                              # Initialise sensitivity of colour detection. Sensitivity is low to avoid flase positives
        self.bridge = CvBridge()                                                          # Initialise a CvBridge()


    def identify_color_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")                            # Convert the received image into a opencv image, wrap
        except CvBridgeError as e:
            print(e)
        
        hsv_green_lower = np.array([75 - self.sensitivity, 100, 100])                     # Set the upper and lower thresholds for each color's identification
        hsv_green_upper = np.array([75 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)                             # Convert the rgb image into a hsv image
       
        greenmask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)              # Mask to filter out everything but green color
        redmask = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)                    # Mask to filter out everything but red color
        imagemask = cv2.bitwise_or(greenmask, redmask)                                    # Combine both masks to produce image that only sees red and green
       
        output_image = cv2.bitwise_and(cv_image, cv_image, mask = imagemask)              # Apply the mask to the original image using the cv2.bitwise_and() method

        gcontours, hierarchy = cv2.findContours(greenmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)      # Create contour for green objects
        rcontours, hierarchy = cv2.findContours(redmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)        # Create contour for red objects
       
        area = 3000         # Area of circle that engulfs desired colors
        range = 2200        # Range for above area (1800-5200)
       
        # Identifying green colors and enclosing them with a white circle
        if len(gcontours) > 0:
            g = gcontours[0]
            for contour in gcontours:
                if cv2.contourArea(contour) > cv2.contourArea(g):
                    g = contour
            M = cv2.moments(g)                                                              # Moments used to calculate the center of the contour
            if M['m00'] != 0:                                                               # When moment is not equal to 0, then try to enclose the colors with a circle
                gx, gy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                if cv2.contourArea(g) > area - range and cv2.contourArea(g) < area + range: # If area of contours is in the range area +- range, color identifier flag is true
                    (gx, gy), green_radius = cv2.minEnclosingCircle(g)                      
                    green_center = (int(gx), int(gy))
                    green_rad = int(green_radius)
                    #                                                   b   g   r
                    cv2.circle(output_image, green_center, green_rad, (255, 255, 0), 5)     # Function parameters are set using previously variables and custom color and thickness
                    self.gflag = True
                else:
                    self.gflag = False                                                      # Switch flag off if color does not fit within contour
        else:
            self.gflag = False                                                              # Switch flag off if color is not visible

        # Identifying red colors and enclosing them with a white circle
        if len(rcontours) > 0:
            r = rcontours[0]
            for contour in rcontours:
                if cv2.contourArea(contour) > cv2.contourArea(r):
                    r = contour
            M = cv2.moments(r)
            if M['m00'] != 0:    
                rx, ry = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                if cv2.contourArea(r) > area - range and cv2.contourArea(r) < area + range:
                    (rx, ry), red_radius = cv2.minEnclosingCircle(r)
                    red_center = (int(rx), int(ry))
                    red_rad = int(red_radius)
                    cv2.circle(output_image, red_center, red_rad, (255, 255, 0), 5)
                    self.rflag = True
                else:
                    self.rflag = False
        else:
            self.rflag = False

        cv2.namedWindow('camera_Feed')                                                      # Display camera
        cv2.imshow('camera_Feed', np.hstack([cv_image, output_image]))                      # Stack original output with what the bot sees after mask and contours are applied
        cv2.waitKey(3)

class Entrance:                                                                             # Class created to perform robust room selection process
    def __init__(self):
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)    # Publish actions to Twist module allowing navigation
        self.rate = rospy.Rate(10)                                                          #10hz
        self.desired_velocity = Twist()
        self.RI = RoomIdentifier()                                                          # Instantiate Room Identifier class to be used in following functions
        self.room1_green = 0
        self.dob_check = False

    def read_input_points(self):
        path = f'{PROJECT_DIR}/world/input_points.yaml'
        with open(path) as opened_file:
            return yaml.safe_load(opened_file)
       
    def move_to_goal(self, goal_pose):                                                      # Move to goal: Sends Turtlebot commands to move to desired location
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)        # Create a client to communicate with the robot navigation system
        move_base_client.wait_for_server()
        goal = MoveBaseGoal()                                                               # Create an object to store the goal state
        goal.target_pose.header.frame_id = 'map'                                            # Set the reference frame to the map
        goal.target_pose.header.stamp = rospy.Time.now()                                    # Set the time when the goal is sent
        goal.target_pose.pose = goal_pose                                                   # Assign the input goal position to the target position
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()                                                  # Wait for the robot to reach the goal state
       
    def begin(self):                                                                        # Begin: Move to first room from origin point
        points = self.read_input_points()
        room1_entrance_xy = points['room1_entrance_xy']
        coord_entrance_room1 = Pose(Point(room1_entrance_xy[0], room1_entrance_xy[1], 0), Quaternion(0, 0, 0, 1))
        self.move_to_goal(coord_entrance_room1)                                             # Navigate to the entrance of room 1
   
    def spin(self):                                                                         # Spin: Turtlebot spins until green or red colors are visible, at which point the robot will stop spinning
        self.desired_velocity.angular.z = 0.5                                               # Rotate with 0.5 m/sec in anti-clockwise direction.
        start_time = time.time()
        while True:                                                                         # Publish spinning motion continuously then stop when green or red colors are detected
            self.pub.publish(self.desired_velocity)                                          
            if self.RI.gflag or self.RI.rflag:                                                  
                self.stop() 
                break
            elapsed_time = time.time() - start_time                                         # if after 15s it has not seen either red or green, stop spinning and execute next action 
            if elapsed_time >= 15:
                break

    def stop(self):                                                                         # Stop: Stop rotating
        self.desired_velocity.angular.z = 0
        self.pub.publish(self.desired_velocity)

    def colour_action(self):                                                                # Colour_Action: Robot enters room if GREEN, or enters other room if RED
        points = self.read_input_points()
        room1_entrance_xy = points['room1_entrance_xy']                                     # Arrange data from input points file into a usable list using read_input_points() function
        room2_entrance_xy = points['room2_entrance_xy']
        room1_centre_xy = points['room1_centre_xy']
        room2_centre_xy = points['room2_centre_xy']
        coord_entrance_room1 = Pose(Point(room1_entrance_xy[0], room1_entrance_xy[1], 0), Quaternion(0, 0, 0, 1))   # Arrange above data into position and orientation data for Turtlebot
        coord_entrance_room2 = Pose(Point(room2_entrance_xy[0], room2_entrance_xy[1], 0), Quaternion(0, 0, 0, 1))
        coord_centre_room1 = Pose(Point(room1_centre_xy[0], room1_centre_xy[1], 0), Quaternion(0, 0, 0, 1))
        coord_centre_room2 = Pose(Point(room2_centre_xy[0], room2_centre_xy[1], 0), Quaternion(0, 0, 0, 1))
        while True:                                                                
            if self.RI.gflag:                                                               # Ths while loop checks the colour of the room 1 entrance
                self.room1_green += 1                                                       # if it is green, the variable room1_green increases and checks the
                self.move_to_goal(coord_entrance_room2)                                     # colour of the room 2. If the room 2 is red, room 1 is even more 
                self.spin()                                                                 # likely to be green, therefore the value of room1_green increases but
                if self.RI.gflag:                                                           # if it is green, green was found in both rooms, there has to be a false  
                    self.room1_green -= 1                                                   # green, therefore the value of room1_green decreases back to 0, uncertainty
                elif self.RI.rflag:                                                          
                    self.room1_green += 1  
                                                                    
                    
            elif self.RI.rflag:                                                             # if red is found in the room 1 entrance, room1_green decreases and goes 
                self.room1_green -= 1                                                       # to check the colour of the room 2 entrance. If the room 2 is green, 
                self.move_to_goal(coord_entrance_room2)                                     # room1_green decreases it value, indicating that most likely the green room 
                self.spin()                                                                 # is not green. If room 2 is red aswell, there is uncertainity so this value 
                if self.RI.gflag:                                                           # will increase to 0, indicating uncertainity.
                    self.room1_green -= 1
                elif self.RI.rflag:
                    self.room1_green += 1
                
            else:
                self.move_to_goal(coord_entrance_room2)
                self.spin()
                if self.RI.gflag:                                                           
                    self.room1_green -= 1
                elif self.RI.rflag:
                    self.room1_green += 1
                
            
            if self.room1_green == 2:                                                       # room1_green value being 2 indicates that room 1 is almost certain to be green
                self.move_to_goal(coord_centre_room1)                                       # therefore room 1 should be entered.
                print("entering green room (room 1)")                                       
                break                                                                       
            elif self.room1_green == -2:                                                    # room1_green value being -2 indicates that room 1 is almost certain to be red
                self.move_to_goal(coord_centre_room2)                                       # therefore room 2 should be entered.
                print("entering green room (room 2)")                                       
                break
            elif self.room1_green == 0:                                                     # room1_green value being 0 indicates that the same colour was found in both 
                if not self.dob_check:                                                      # room entrances or no colour was found in any of them. Due to the uncertainity
                    self.dob_check = True                                                   # the robot will double check both entrances and the dob_check bool will turn True,
                    self.move_to_goal(coord_entrance_room1)                                 # in order to not double check again, once the dob_check is True, and the room1_green
                    self.spin()                                                             # value is 0, the robot will just go to room 2. 
                    continue
                else:
                    self.move_to_goal(coord_centre_room2)                                   
                    print("entering room 2")
                    print(self.room1_green)
                    break
            else:
                if self.room1_green > 0:                                                    # There are cases where room1_green value is not 2 nor -2 nor 0, in this case if it 
                    self.move_to_goal(coord_centre_room1)                                   # has any negative value, the likelihood that room 1 is green is lower than being red
                    print("entering room 1")                                                # therefore the robot will go to room 2.
                    print(self.room1_green)
                    break
                else:                                                                       # Viceversa for a positive value of room1_green.
                    self.move_to_goal(coord_centre_room2)                                   
                    print("entering room 2")
                    print(self.room1_green)
                    break
 
    def search(self):                           # Search: Function to combine all previous functions into one action (essentially the 'main')
        self.begin()                            # Begin by moving to starting position: Entrance of Room 1
        self.spin()                             # Rotate until either green or red is visible
        self.colour_action()                    # Respond to visible colors accordingly


if __name__ == "__main__":                      # If this file is run as main, perform the search() function only. THe result of this is the Turtlebot idle in the centre of the green room
    rospy.init_node('Entrance', anonymous=True)
    entrance = Entrance()
    entrance.search()