import rospy
import yaml
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import cv2
import sys

# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Entrance_module import *
from MotionPlanning import *

def read_input_points():
    path = f'{PROJECT_DIR}/world/input_points.yaml'
    with open(path) as opened_file:
        return yaml.safe_load(opened_file)


def write_character_id(character_id):
    path = f'{PROJECT_DIR}/output/cluedo_character.txt'
    with open(path, 'w') as opened_file:
        opened_file.write(character_id)


def save_character_screenshot(cv_image):
    path = f'{PROJECT_DIR}/output/cluedo_character.png'
    cv2.imwrite(os.path.expanduser(path, cv_image))


def move_to_goal(goal_pose):
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = goal_pose
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('group_project')
    entrance = Entrance()
    entrance.search()

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

        
    


    # Perform any additional actions, such as identifying characters, saving screenshots, etc.
    # For example:
    # write_text_file('mustard')
    # save_character_screenshot(cv_image)