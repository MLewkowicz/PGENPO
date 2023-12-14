Software setup: 
Required packages + environments: 
Docker container with MoFA pre-trained autoencoder mounted on shared volume: 
Sci-kit learn

Relevant Scripts description: 
roslaunch rs_camera.launch (brings up the realsense camera)
(ROS) rosrun ur_robot_utils execute_trajectories.py (runs a selected trajectory on the UR5 robot and/or runs PGENPO demo)
(ROS) python face_detection.py (looks for face in camera frame. Starts the calibration step)
Demo.py: Runs inference with pre-trained MoFA network on newly collected images
Scp_listener.py: Listens for newly written folders with camera images on the LINUX machine. 
React app (front-end interface)
Hardware setup: 
URD5 robot
Realsense camera, 3D mounted in line with the gripper of the UR5 robot. (*gripper must be closed to avoid self collisions) 
One LINUX machine running Ubuntu 18:04 which interfaces with the MoveIT controller to execute trajectories on the UR5 robot, and Windows machine to run MoFA and PGENPO inference. 

General Flow for running a trial:
roslaunch control_wrapper ur5e_cam_2f85_control.launch
Start up face detection and trajectory script on linux machine 1. (section 1) 
Start up file-systems listeners to feed into MoFa inference on windows machine 2. (section 2) 
Start up web-app display to get live feedback of camera stream and PGENPO inferences

Ros Commands to set up Camera Stream 
Confirm that camera feed from real-sense camera is functional
Rosrun — (Face detection) 
Confirm that face bounding boxes are being detected. 
Rosrun ur_robot_utils execute_trajectories 

Commands to set up MoFA stream and web-app display
flask run app.py
npm start (for react front-end)
Debugging commands: 
rosrun ur_robot_utils execute_trajectories: this allows you to select pre-recorded trajectories and run them. Useful to see the speed and waypoints of trajectories already recorded
