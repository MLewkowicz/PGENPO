#! /usr/bin/env python

import rospy
import numpy as np

from gripper_control import Gripper

# Message imports go here
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import control_msgs
import trajectory_msgs
from trajectory_msgs.msg import *
from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
    JointTrajectoryAction
)

# Service imports go here
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from control_wrapper.srv import GetJoints

# All other imports go here
import os
import sys
import pickle as pkl
import rospkg

import actionlib
import time
import roslib

roslib.load_manifest('ur_driver')

# Hyper-parameters go here
SLEEP_RATE = 10
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GRIPPER_CMD = "gripper_open"
TTS_CMD = "tts"

PKG_NAME = "ur_robot_utils"

IMAGE_PATH = "/home/scazlab/catkin_ws/src/pgenpo/bags"

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
    
class ExecuteTrajectory(object):
    def __init__(self, fn="traj_dict_path.pkl"):
        self.camera_topic = "/camera/color/image_raw"
        self.offset_topic = "/face_center_offset"
        self.offsets = []
        
        rospack = rospkg.RosPack()
        save_dir = "trajectories"
        pkg_path = rospack.get_path(PKG_NAME)
        self._save_path = os.path.join(pkg_path, save_dir)
        self.traj_dict_path = os.path.join(self._save_path, fn)

        self.client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_position_dict = self._load_trajectory()  # {'trajectory1_name': [{joint1_name: joint1_state, joint2_name: joint2_state, ...},
                                                            #                      {joint1_name: joint1_state, joint2_name: joint2_state, ...},
                                                            #                       .............],
                                                            # 'trajectory2_name': [{joint1_name: joint1_state, joint2_name: joint2_state, ...},
                                                            #                      {joint1_name: joint1_state, joint2_name: joint2_state, ...},
                                                            #                       .............]
        self.gripper = Gripper()
        rospy.wait_for_service("/ur/control_wrapper/left/get_joints")
        
        self._get_cur_joints_sp = rospy.ServiceProxy("/ur/control_wrapper/left/get_joints", GetJoints)
        

    def _load_trajectory(self):
        ret_dict = None

        with open(self.traj_dict_path, "rb") as f:
            ret_dict = pkl.load(f)

        return ret_dict

    def _select_trajectories(self):
        """Keyboard interface for selecting one of the saved trajectories"""
        traj_by_num = dict(enumerate(self.joint_position_dict))
        traj_chosen = False
        while not traj_chosen:
            print("Available trajectories: ")
            for k in traj_by_num:
                print("\t{}: {}".format(k, traj_by_num[k]))

            try:
                # choice = int(input("Enter trajectory number (s): "))
                choice_str = input("Enter trajectory number (s): ").strip()
                choice = [int(c) for c in choice_str.split()]
                ret = [traj_by_num[c] for c in choice]
                traj_chosen = True
            except KeyError:
                print("{} not a valid trajectory choice, try again!\n".format(choice))
            except ValueError:
                print("Not a number, try again!\n")

        return ret

    def _execute_trajectory(self, traj_name, check_height=False):
        START_DUR = 0.0
        MAX_VEL = 191 # degree per sec.
        TIME_BUFF = .3 # sec
        cur_speed = MAX_VEL * .25 # CHECK TABLET FOR TRUE SCALING VAL.
        cur_joints = self._get_cur_joints_sp()
        cur_joints = np.array(cur_joints.joints.position)
        # extract joint names and angles from the dict # Select a trajectory
        # from cmdline input and retrieve corresponding data.
        traj = self.joint_position_dict[traj_name]
        cmd_list = []   # Will store joint cmds transformed into ros data

        # joint_velocities = [0.01] * len(JOINT_NAMES)
        # joint_accelerations = [0.0] * len(JOINT_NAMES)
        # We must break the overall trajectory into sub trajectories
        # As cmds to the gripper are issued through a completely diff
        # process. These values help keep track of this.
        start_time = START_DUR   
        new_sub_traj = True
        traj_idx = 0
        for i, cmd in enumerate(traj):
            # If its a gripper cmd and not a postition, just append to list
            # and we'll execute it later
            # if GRIPPER_CMD in cmd or TTS_CMD in cmd:
            if GRIPPER_CMD in cmd or TTS_CMD in cmd:
                cmd_list.append(cmd)
                new_sub_traj = True
                traj_idx = len(cmd_list)
                start_time = 1
            else:
                if new_sub_traj:
                    cmd_list.append([])
                    new_sub_traj = False
                    start_time = START_DUR   #

                positions = [cmd[j] for j in JOINT_NAMES]
                new_joints = np.array(positions)
                max_deg_dist = max(abs(new_joints - cur_joints))
                joint_velocities = np.array([0]*6) # 1.4
                joint_accelerations = np.array([0]*6) # 4.4
                # print("max_deg_dist: ", max_deg_dist)
                # print("calc time: ", np.rad2deg(max_deg_dist / cur_speed))
                start_time += np.rad2deg(max_deg_dist / cur_speed) + TIME_BUFF # t = d / v

                cmd_list[traj_idx].append(
                    JointTrajectoryPoint(positions=positions,
                                         velocities=joint_velocities,
                                         accelerations=joint_accelerations,
                                         time_from_start=rospy.Duration(
                                             start_time)
                                         ))
                cur_joints = new_joints
            # start_time += 1.0
        if not check_height:    
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
        else:
            self.offsets = []
        
        for i, cmd in enumerate(cmd_list):
            if isinstance(cmd, list):
                g = FollowJointTrajectoryGoal()
                g.trajectory = JointTrajectory()
                g.trajectory.joint_names = JOINT_NAMES
                g.trajectory.points = cmd
                
                print("Trajectory points {}".format(g.trajectory.points))
                rospy.loginfo("Executing sub trajectory!")
                for i, p in enumerate(cmd):
                    if i <= 10:
                        sub_traj = FollowJointTrajectoryGoal()
                        sub_traj.trajectory = JointTrajectory()
                        sub_traj.trajectory.joint_names = JOINT_NAMES
                        sub_traj.trajectory.points = [p]
                        
                        self.client.send_goal(sub_traj)
                        self.client.wait_for_result()

                        print("point {} time: {}".format(i, p.time_from_start))
                        
                        if not check_height:
                            camera_msg = rospy.wait_for_message(self.camera_topic, Image)

                            # save single camera image at each waypoint
                            try:
                                cv_image = imgmsg_to_cv2(camera_msg)
                                print(cv_image.shape)
                            except CvBridgeError as e:
                                print(e)
                            
                            # Save the image
                            image_path_str = "waypoint_" + str(i) + ".jpg"
                            image_path = os.path.join(new_trial_folder, image_path_str)

                            cv2.imwrite(image_path, cv_image)
                            rospy.loginfo(f"Saved image to {image_path}")
                        else:
                            offset_msg = rospy.wait_for_message(self.offset_topic, Float32)
                            self.offsets.append(offset_msg.data)
                            
                    

            elif GRIPPER_CMD in cmd:
                rospy.loginfo("Commanding the gripper!")
                if cmd[GRIPPER_CMD]:
                    self.gripper.open_gripper()
                else:
                    self.gripper.close_gripper()

            # NOTE: This sleep is REQUIRED for robot to reconnect ROS
            # afer using the gripper.
            rospy.sleep(.5)

    def run(self):
        traj_names = self._select_trajectories()

        for traj in traj_names:
            self._execute_trajectory(traj)

        # self.clip_5()

    def _execute_traj_by_num(self, traj_idx_list):
        """
        Execute recorded trajectories indicated by traj_idx_list
        @traj_idx_list: list of ints, where ints correspond to recorded
        trajectory in self.joint_position_dict
        """
        traj_by_num = dict(enumerate(self.joint_position_dict))
        for traj_idx in traj_idx_list:
            # input("Press enter to continue...")
            traj = traj_by_num[traj_idx]
            rospy.loginfo("Executing {}".format(traj))
            self._execute_trajectory(traj)

    def example_usage(self):
        traj_num_list = [15, 17, 2, 0, 10, 17, 12, 13, 14 ] # TODO: Put traj numbers in
        #traj_num_list = [14]
        self._execute_traj_by_num(traj_num_list)
    
    def pgenpo_demo(self):
        traj_by_num = dict(enumerate(self.joint_position_dict))
        height_trajectory = traj_by_num[55]
        
        self._execute_trajectory(height_trajectory, check_height=True)
        
        
        if len(self.offsets) == 2:
            short_trajectory = traj_by_num[54]
            tall_trajectory = traj_by_num[56]
            if self.offsets[0] < self.offsets[1]:
                self._execute_trajectory(tall_trajectory, check_height=False)
            else:
                self._execute_trajectory(short_trajectory, check_height=False)
        

def main():
    rospy.init_node("execute_trajectory")

    fn = "trajectories.pkl"
    execute_trajectory = ExecuteTrajectory(fn)
    print("Waiting for server...")
    execute_trajectory.client.wait_for_server()
    print("Connected to server...")

    rate = rospy.Rate(SLEEP_RATE)

    # This while loop is what keeps the node from dying
    while not rospy.is_shutdown():
        # execute_trajectory.run()
        execute_trajectory.pgenpo_demo()
        rate.sleep()


if __name__ == '__main__':
    main()
