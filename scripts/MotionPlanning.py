import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import threading
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Entrance_module import *

PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')

class Robot:
    
    def __init__(self):
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

    
        
        
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.rate = rospy.Rate(10) #10hz
        self.desired_velocity = Twist()
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.last_pose = None


        self.spin_timer = time.time()
        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Initialize the reference images
        ref_image_mustard = cv2.imread(f'{PROJECT_DIR}/cluedo_images/mustard.png')
        ref_image_peacock = cv2.imread(f'{PROJECT_DIR}/cluedo_images/peacock.png')
        ref_image_plum = cv2.imread(f'{PROJECT_DIR}/cluedo_images/plum.png')
        ref_image_scarlett = cv2.imread(f'{PROJECT_DIR}/cluedo_images/scarlet.png')

        # Initialize the reference descriptors list
        self.ref_descriptors_list = []
        orb = cv2.ORB_create(scoreType=cv2.ORB_HARRIS_SCORE, edgeThreshold=150)

        # Compute descriptors for each reference image and add them to the list
        for ref_image in [ref_image_mustard, ref_image_peacock, ref_image_plum, ref_image_scarlett]:
            gray_ref = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
            _, des_ref = orb.detectAndCompute(gray_ref, None)
            self.ref_descriptors_list.append(des_ref)

        self.orb = orb
        self.ref_images = [ref_image_mustard, ref_image_peacock, ref_image_plum, ref_image_scarlett]
        
    

    def laser_callback(self, msg):
        # Check if there are any obstacles within 0.5 meters
        min_range = min(msg.ranges)
        if min_range < 0.5:

            self.obstacle_detected = True
            self.obstacle_direction = msg.ranges.index(min_range)
        else:
            self.obstacle_detected = False
            self.obstacle_direction = None

     


    
    def MotionCallback(self, msg):
        
     
        # Convert the image to a OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

       #  # Define the color ranges in HSV
        blue_lower = np.array([100, 50, 50])
        blue_upper = np.array([130, 255, 255])
        
        red_lower = np.array([0, 150, 150])
        red_upper = np.array([10, 196, 189])

        purple_lower = np.array([310/2, 0.4*255, 0.2*255])
        purple_upper = np.array([320/2, 1*255, 1*255])
        
        yellow_lower = np.array([20, 140, 100])
        yellow_upper = np.array([72, 221, 200])

        # Threshold the image for each color
        blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
        red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
        purple_mask = cv2.inRange(hsv_image, purple_lower, purple_upper)
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        
        # Calculate moments of the blue mask
        M_blue = cv2.moments(blue_mask)
        M_red= cv2.moments(red_mask)
        M_yellow = cv2.moments(yellow_mask)
        M_purple = cv2.moments(purple_mask)
        

        # Check if blue color is detected
        if M_blue["m00"] > 0:
            print("Moving towards blue")
            # Calculate centroid of the blue mask
            cx = int(M_blue["m10"] / M_blue["m00"])
            cy = int(M_blue["m01"] / M_blue["m00"])

            # Determine direction to move towards the centroid of the blue mask
            width, height = cv_image.shape[:2]
            if cx > width/2:
                # Blue is to the right of the center, turn left
                self.desired_velocity.angular.z = -0.5
            else:
                # Blue is to the left of the center, turn right
                self.desired_velocity.angular.z = 0.5

            # Move forward towards the blue color
            self.desired_velocity.linear.x = 0.2
            
        # Check if blue color is detected
        if M_red["m00"] > 0:
            print("Moving towards red")
            # Calculate centroid of the blue mask
            cx = int(M_red["m10"] / M_red["m00"])
            cy = int(M_red["m01"] / M_red["m00"])

            # Determine direction to move towards the centroid of the red mask
            width, height = cv_image.shape[:2]
            if cx > width/2:
                # redis to the right of the center, turn left
                self.desired_velocity.angular.z = -0.5
            else:
                # red is to the left of the center, turn right
                self.desired_velocity.angular.z = 0.5

            # Move forward towards the red color
            self.desired_velocity.linear.x = 0.2
            
            
            
        # Check if purple color is detected  
        if M_purple["m00"] > 0:
            print("Moving towards purple")
            # Calculate centroid of the purple mask
            cx = int(M_purple["m10"] / M_purple["m00"])
            cy = int(M_purple["m01"] / M_purple["m00"])

            # Determine direction to move towards the centroid of the purple mask
            width, height = cv_image.shape[:2]
            if cx > width/2:
                # purple is to the right of the center, turn left
                self.desired_velocity.angular.z = -0.5
            else:
                # purple is to the left of the center, turn right
                self.desired_velocity.angular.z = 0.5

            # Move forward towards the purple color
            self.desired_velocity.linear.x = 0.2
            
                

        # Check if yellow color is detected  
        if M_yellow["m00"] > 0:
            print("Moving towards yellow")
            # Calculate centroid of the yellow mask
            cx = int(M_yellow["m10"] / M_yellow["m00"])
            cy = int(M_yellow["m01"] / M_yellow["m00"])

            # Determine direction to move towards the centroid of the yellow mask
            width, height = cv_image.shape[:2]
            if cx > width/2:
                # yellow is to the right of the center, turn left
                self.desired_velocity.angular.z = -0.5
            else:
                # yellow is to the left of the center, turn right
                self.desired_velocity.angular.z = 0.5

            # Move forward towards the yellow color
            self.desired_velocity.linear.x = 0.2
            
     
        self.pub.publish(self.desired_velocity)
        if(self.obstacle_detected == True):
            rospy.sleep(3)
            self.rate.sleep()

    
        
        
        
    #move robot forward
    def move_forward(self):
        # Move forward until an obstacle is detected
        self.desired_velocity.linear.x = 0.2
        self.desired_velocity.angular.z = 0
        while not self.obstacle_detected:
            self.pub.publish(self.desired_velocity)
            self.rate.sleep()
        self.desired_velocity.linear.x = 0
        self.pub.publish(self.desired_velocity)



    def avoid_obstacle(self):
        # Move back
        self.desired_velocity.linear.x = -0.25
        self.desired_velocity.angular.z = 0.3
        
        for _ in range(10):
            self.pub.publish(self.desired_velocity)
            self.rate.sleep()   
        # GO back and turn a bit 
        self.desired_velocity.linear.x = -0.3
        self.desired_velocity.angular.z = 0.3
        # Stop the robot
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0
        self.pub.publish(self.desired_velocity)
        # Turn away from the obstacle
        if self.obstacle_direction is not None:
            angle_to_obstacle = math.radians(self.obstacle_direction)
            if angle_to_obstacle < math.pi:
                self.desired_velocity.angular.z = 0.6
            else:
                self.desired_velocity.angular.z = -0.6
            # Turn for 1 second
            for _ in range(2):
                self.pub.publish(self.desired_velocity)
                self.rate.sleep()
            # Stop turning
            self.desired_velocity.angular.z = 0
            self.pub.publish(self.desired_velocity)

    def spin(self):
        # Spin 360 degrees
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 1.66
        for _ in range(40):
            self.pub.publish(self.desired_velocity)
            self.rate.sleep()

    def runmotion(self):
        
        self.color_sub= rospy.Subscriber("/camera/rgb/image_raw", Image, self.MotionCallback)
        last_spin_time = time.time()
        self.spin()  # spin first
        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - last_spin_time >= 5:
                self.spin()
                last_spin_time = current_time
            self.move_forward()
            if self.obstacle_detected:
                self.avoid_obstacle()
                self.obstacle_detected = False
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0
        self.pub.publish(self.desired_velocity)
    
    def runFeature(self):
           while not rospy.is_shutdown():
              # Start the main loop
              rospy.Subscriber("/camera/rgb/image_raw", Image, self.FeatureCallback)
              rospy.spin()
        
        
        
        
        
        
#***********************************************************************************************************************************************************/



    def detect_face(self,frame):
        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Load the pre-trained Haar Cascade classifier for face detection
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Apply face detection using the classifier
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=8, minSize=(30, 30))

        # If any faces are detected, return True, otherwise return False
        if len(faces) > 0:
            return True
        else:
            return False




        
    def orb_detection(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Create an ORB detector object with increased threshold
        orb = cv2.ORB_create(scoreType=cv2.ORB_HARRIS_SCORE, edgeThreshold=150)

        # Detect key points and compute their descriptors using ORB algorithm
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Draw the detected key points on the original image
        image = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0), flags=0)

        # Compare the descriptors of the live image with the descriptors of the reference images
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        num_matches_list = []
        for ref_descriptors in self.ref_descriptors_list:
            matches = bf.match(ref_descriptors, descriptors)
            num_matches = len(matches)
            num_matches_list.append(num_matches)
            # Print the number of matches for this reference image
            print("Number of matches for reference image:", num_matches)

        # Return the modified image, descriptors, and number of matches
        return image, descriptors, num_matches_list

    def FeatureCallback(self,msg):
        try:
            # Convert the image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform ORB feature detection on the image
            orb_result, descriptors, num_matches = self.orb_detection(frame)
            detect_face_result = self.detect_face(frame)

            # Convert the frame from BGR to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the blue color range in HSV
            blue_lower = np.array([100, 50, 50])
            blue_upper = np.array([130, 255, 255])

            # Define the red color range in HSV
            red_lower = np.array([0,50,50])
            red_upper = np.array([10,255,255])

            # define range of purple color in HSV
            purple_lower = np.array([310/2, 0.4*255, 0.2*255])
            purple_upper = np.array([320/2, 1*255, 1*255])


            # Define the yellow color range in HSV
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])

            # Threshold the image to detect blue color
            blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
            blue_result = cv2.bitwise_and(frame, frame, mask=blue_mask)

            # Threshold the image to detect red color
            red_mask = cv2.inRange(hsv, red_lower, red_upper)
            red_result = cv2.bitwise_and(frame, frame, mask=red_mask)

            # Threshold the image to detect purple color
            purple_mask = cv2.inRange(hsv, purple_lower, purple_upper)
            purple_result = cv2.bitwise_and(frame, frame, mask=purple_mask)

            # Threshold the image to detect yellow color
            yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
            yellow_result = cv2.bitwise_and(frame, frame, mask=yellow_mask)

            # Determine the color detected and print it to the screen
            if cv2.countNonZero(blue_mask) > 0 and any(match > 85 for match in num_matches) and detect_face_result == True :
                print(detect_face_result)
                print("Peacock")
                self.write_character_id("Mrs Peacock")
                self.save_character_screenshot(frame)
                rospy.signal_shutdown("Took Screenshot, Shutting Down!")
                
            elif cv2.countNonZero(yellow_mask) > 0 and any(match > 85 for match in num_matches) and detect_face_result== True:
                print(detect_face_result)
                print("Mustard")
                self.write_character_id("Colonel Mustard")
                self.save_character_screenshot(frame)
                rospy.signal_shutdown("Took Screenshot, Shutting Down!")
            
            elif cv2.countNonZero(purple_mask) > 0 and any(match > 85 for match in num_matches) and detect_face_result == True :
                print(detect_face_result)
                print("Plum")
                self.write_character_id("Professor Plum")
                self.save_character_screenshot(frame)
                rospy.signal_shutdown("Took Screenshot, Shutting Down!")

            elif cv2.countNonZero(red_mask) > 0 and any(match > 85 for match in num_matches) and detect_face_result == True:
                print(detect_face_result)
                print("Scarlett")
                self.write_character_id("Miss Scarlett")
                self.save_character_screenshot(frame)
                rospy.signal_shutdown("Took Screenshot, Shutting Down!")
                

            else:
                print("No color or feature detected")

            # Display the results
            cv2.imshow('Color detection - blue', blue_result)
            cv2.imshow('Color detection - red', red_result)
            cv2.imshow('Color detection - purple', purple_result)
            cv2.imshow('Color detection - yellow', yellow_result)
            cv2.imshow('ORB feature detection', orb_result)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
    
    def write_character_id(self,character_id):
        path = f'{PROJECT_DIR}/output/cluedo_character.txt'
        with open(path, 'w') as opened_file:
            opened_file.write(character_id)
    
    
    def save_character_screenshot(self, cv_image):
        path = f'{PROJECT_DIR}/output/cluedo_character.png'
        expanded_path = os.path.expanduser(path)
        cv2.imwrite(expanded_path, cv_image)

    
    
    
def runmotion_wrapper(robot):
    robot.runmotion()

def runFeature_wrapper(robot):
    robot.runFeature()


   

        
if __name__ == '__main__':
    rospy.init_node('group_project')
    

    robot = Robot()  # instantiate your robot object here

    # create two threads, one for each function
    motion_thread = threading.Thread(target=runmotion_wrapper, args=(robot,))
    feature_thread = threading.Thread(target=runFeature_wrapper, args=(robot,))

    # start the threads
    motion_thread.start()
    feature_thread.start()

    # wait for the threads to finish
    motion_thread.join()
    feature_thread.join()
    