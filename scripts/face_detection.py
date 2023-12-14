import rospy
import numpy as np
import cv2
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yoloface import face_analysis

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
    
# Callback function to process the incoming camera image
def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        try:
            frame = imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
        
        # Perform face detection
        _, box, conf = face.face_detection(frame_arr=frame, frame_status=True, model='tiny')
        
        # Calculate the vertical offset between the box and the center of the camera frame
        if box is not None:
            bbox = box[0]
            # Get the coordinates of the center of the camera frame
            camera_center_x = frame.shape[1] // 2
            camera_center_y = frame.shape[0] // 2
            print("Camera Center Y: {}".format(camera_center_y))
            # Get the coordinates of the center of the detected face box
            box_center_y = (bbox[1] + bbox[2] // 2)
            
            # Calculate the vertical offset
            vertical_offset = abs(box_center_y - camera_center_y)
            
            # Print the vertical offset
            print("Vertical Offset: {}".format(vertical_offset))
            
            height_offset_pub.publish(vertical_offset)
            
        
        # Show the output frame
        output_frame = face.show_output(frame, box, frame_status=True)
        
        cv2.imshow('frame', output_frame)
        face_bbox_pub.publish(cv2_to_imgmsg(output_frame))
        # Check for key press to exit
        key = cv2.waitKey(1)
        if key == ord('v'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Exit requested by user")
    
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('yoloface_node')
    
    # Set up the subscriber for the camera image topic
    camera_img_topic = "/camera/color/image_raw"  # Replace with your actual camera image topic
    face_offset = "/face_center_offset"
    rospy.Subscriber(camera_img_topic, Image, image_callback)
    height_offset_pub = rospy.Publisher("/face_center_offset", Float32, queue_size=10)
    face_bbox_pub = rospy.Publisher("/face_detection", Image, queue_size=10)
    # Initialize YOLOFace
    face = face_analysis()
    # Main ROS loop
    rospy.spin()
    