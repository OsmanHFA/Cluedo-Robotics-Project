import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np



class ImageIdentifier:
    def __init__(self):

        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Initialize ORB detector object
    orb = cv2.ORB_create()

        # Load the reference image of the Cluedo character
    #ref_image = cv2.imread("/home/csunix/el19ofa/catkin_ws/src/lab3/scripts/Cluedo_character.png")
    ref_image = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
    ref_keypoints, ref_descriptors = orb.detectAndCompute(ref_image, None)


    def detect_face(self,frame):
        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Load the pre-trained Haar Cascade classifier for face detection
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Apply face detection using the classifier
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # If any faces are detected, return True, otherwise return False
        if len(faces) > 0:
            return True
        else:
            return False




        
    def orb_detection(self,image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Create an ORB detector object with increased threshold
        orb = cv2.ORB_create(scoreType=cv2.ORB_HARRIS_SCORE, edgeThreshold=150)

        # Detect key points and compute their descriptors using ORB algorithm
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Draw the detected key points on the original image
        image = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0), flags=0)

        # Compare the descriptors of the live image with the descriptors of the reference image
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self.ref_descriptors, descriptors)
        num_matches = len(matches)

        # Print the number of matches to the screen
        print("Number of matches:", num_matches)

        # Return the modified image, descriptors, and number of matches
        return image, descriptors, num_matches

    def image_callback(self,msg):
        try:
            # Convert the image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform ORB feature detection on the image
            orb_result, self.descriptors, num_matches = self.orb_detection(frame)
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
            if cv2.countNonZero(blue_mask) > 0 and num_matches > 90 and detect_face_result == True :
                print(detect_face_result)
                print("peackcock")
                
            elif cv2.countNonZero(yellow_mask) > 0 and num_matches > 90 and detect_face_result== True:
                print(detect_face_result)
                print("Mustard")
            
            elif cv2.countNonZero(purple_mask) > 0 and num_matches > 90 and detect_face_result == True :
                print(detect_face_result)
                print("Plum")

            elif cv2.countNonZero(red_mask) > 0 and num_matches > 90 and detect_face_result == True:
                print(detect_face_result)
                print("Scarlett")

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


    def run(self):
       
        while not rospy.is_shutdown():
              # Start the main loop
              rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
              rospy.spin()
        
        
    
if __name__ == '__main__':
    rospy.init_node('group_project')
 
    
    CluedoIdentifier = ImageIdentifier()
    CluedoIdentifier.run()
    
  