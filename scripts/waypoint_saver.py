#!/usr/bin/env python

import rospy  # If you are doing ROS in python, then you will always need this import
import message_filters
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError


# Message imports go here
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# Service imports go here

# All other imports go here


# Hyper-parameters go here
SLEEP_RATE = 10
IMAGE_PATH = '/home/scrc/PGENPO/MoFA/Data/Results'

def cv2_to_imgmsg(cv_image):
        '''
        Helper function to publish a cv2 image as a ROS message (without using ROS cv2 Bridge)
        https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/
        Parameters:
                    cv_image (image): Image to publish to a ROS message
        Returns:
                    img_msg (message): Image message published to a topic
        '''
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

def imgmsg_to_cv2(img_msg):
        '''
        Helper function to convert a ROS RGB Image message to a cv2 image (without using ROS cv2 Bridge)
        https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/

        Parameters:
                    img_msg (message): Image message published to a topic 
        Returns:
                    cv_image (image): cv2 image
        '''
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_BGR2RGB)

        return image_opencv
    
    
class WayPointSaver(object):
    def __init__(self):

        # Publishers go here
        self.waypoint_pub = rospy.Publisher('/take_picture/response', Bool)
        # Service Proxies go here

        # Subscribers go here
        self.waypoint_sub = rospy.Subscriber('/take_picture/request', Bool, callback=self.save_waypoint)
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image)
    
    
    def save_waypoint(self, msg):
        camera_msg = rospy.wait_for_message(self.camera_topic, Image)

        # save single camera image at each waypoint
        try:
            cv_image = imgmsg_to_cv2(camera_msg)
            print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        # Save the image
        image_path_str = "waypoint_" + str(i) + ".jpg"
        
        # Ensure the base directory exists
        if not os.path.exists(IMAGE_PATH):
            os.makedirs(IMAGE_PATH)

        # List all subdirectories (trial folders)
        trial_folders = [f for f in os.listdir(IMAGE_PATH) if os.path.isdir(os.path.join(IMAGE_PATH, f))]

        # If no trial folders exist, create trial_0
        if not trial_folders:
            new_trial_folder = os.path.join(IMAGE_PATH, 'trial_0')
        else:
            # Find the highest numbered trial folder and increment it
            highest_label = max([int(folder.split('_')[-1]) for folder in trial_folders])
            new_label = highest_label + 1
            new_trial_folder = os.path.join(IMAGE_PATH, f'trial_{new_label}')

        # Create the new trial folder
        os.makedirs(new_trial_folder)
        print(f"Created new trial folder: {new_trial_folder}")
        
        image_path = os.path.join(new_trial_folder, image_path_str)

        cv2.imwrite(image_path, cv_image)
        rospy.loginfo(f"Saved image to {image_path}")
        
        response_msg =  Bool()
        response_msg.data = True
        
        self.waypoint_pub.publish(response_msg)
        

def main():
    rospy.init_node("waypoint_saver")

    waypoint_saver = WayPointSaver()

    rate = rospy.Rate(SLEEP_RATE)

    # This while loop is what keeps the node from dieing
    while not rospy.is_shutdown():
        # If I wanted my node to constantly publish something, I would put the publishing here
        rate.sleep()


if __name__ == '__main__':
    main()
