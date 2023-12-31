#!/usr/bin/env python

import os
import rospy
import rospkg
import pickle as pkl
import cv2
from cv_bridge import CvBridge, CvBridgeError
from collections import defaultdict
from gripper_control import Gripper

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Bool

from control_wrapper.srv import Reset, ResetRequest



PKG_NAME = "ur_robot_utils"
GRIPPER_OPEN = "go"
GRIPPER_CLOSED = "gc"
DEFAULT_POSITION = "d"
SAVE = "s"
DETELTE = "del"


class PositionRecorder(object):
    def __init__(self, fn="traj_dict_path.pkl"):
        "docstring"
        rospack = rospkg.RosPack()
        save_dir = "trajectories"
        pkg_path = rospack.get_path(PKG_NAME)
        self._save_path = os.path.join(pkg_path, save_dir)
        self.traj_dict_path = os.path.join(self._save_path, fn)

        self.gripper = Gripper()
        self.topic = "/ur/control_wrapper/left/"
        self.camera_topic = "/camera/color/image_raw"
        self.free_drive_pub = rospy.Publisher(self.topic + "enable_freedrive", Bool, queue_size=10)

        self.traj_dict = self.load_trajectory_dict()

        # The rate at which the node will automatically sample trajectory points
        # self._sample_rate = rospy.get_param("demo/sample_rate", default=.2)

        rospy.loginfo("pkg path: " +  pkg_path)
        rospy.loginfo("save path: " +  self._save_path)

        self.image_save_directory = "waypoint_snapshots"
        if not os.path.exists(self.image_save_directory):
            os.makedirs(self.image_save_directory)
        self.bridge = CvBridge()

    def set_default_angles(self):
        rospy.wait_for_service(self.topic + "reset")
        reset = rospy.ServiceProxy(self.topic + "reset", Reset)
        print(Reset)
        try:
            response = reset().is_reached
        except rospy.ServiceException as exc:
            print(("Service did not process request: " + str(exc)))

    def load_trajectory_dict(self):
        ret_dict = defaultdict(list)

        if not os.path.isdir(self._save_path):
            os.mkdir(self._save_path)

        if os.path.isfile(self.traj_dict_path):
            with open(self.traj_dict_path, "rb") as f:
                ret_dict = pkl.load(f)

        return ret_dict


    def run(self):
        done = False

        print("Current trajectories:")
        for t in self.traj_dict:
            print("\t{}".format(t))

        traj_name = input("Please provide a name for this trajectory: ").strip()
        prompt_str = """
        Keyboard controls:\n\td: Move to the default position (recommended initially)
        Enter: Get point\n\tgo: gripper open\n\tgc: gripper closed\n\tq: End recording.
        s: save current trajectory to file.
        del: Delete the selected trajectory if it exist.
        x: Enter an TTS commands.
        """
        self.free_drive_pub.publish(True)
        while not done:
            self.free_drive_pub.publish(True)
            in_str = input(prompt_str).strip()
            if in_str == GRIPPER_OPEN:
                rospy.loginfo("Opening gripper")
                self.gripper.open_gripper()

                self.traj_dict[traj_name].append({"gripper_open": True})

            elif in_str == GRIPPER_CLOSED:
                rospy.loginfo("Close gripper")
                self.gripper.close_gripper()

                self.traj_dict[traj_name].append({"gripper_open": False})

            elif in_str == DEFAULT_POSITION:
                rospy.loginfo("Going to default position")
                self.set_default_angles()
                self.gripper.open_gripper()

            elif in_str == SAVE:
                rospy.loginfo("Saving...")
                with open(self.traj_dict_path, "wb") as f:
                    pkl.dump(self.traj_dict, f, protocol=pkl.HIGHEST_PROTOCOL )

            elif in_str == DETELTE:
                rospy.loginfo("Deleting {}...".format(traj_name))
                try:
                    self.traj_dict.pop(traj_name)
                except KeyError:
                    rospy.loginfo("{} not a valid trajectory".format(traj_name))

            else:
                rospy.loginfo("Getting point")
                msg = rospy.wait_for_message("/joint_states", JointState)
                joint_names = msg.name
                position = msg.position

                self.traj_dict[traj_name].append(dict(zip(joint_names, position)))

            print(self.traj_dict)



if __name__ == '__main__':
    rospy.init_node("position_recoder")
    fn = "trajectories.pkl"
    pr = PositionRecorder(fn)

    while not rospy.is_shutdown():
        pr.run()
        rospy.sleep(.1)
