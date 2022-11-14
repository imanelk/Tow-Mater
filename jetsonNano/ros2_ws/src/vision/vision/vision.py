# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

from interfaces.msg import Hook
from sensor_msgs.msg import Image # Image is the message type

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

import matplotlib
import numpy as np
import imutils

#colors
from webcolors import rgb_to_name,CSS3_HEX_TO_NAMES,hex_to_rgb #python3 -m pip install webcolors
from scipy.spatial import KDTree


class Vision(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('vision')
            
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_hook_ = self.create_publisher(Hook, 'hook', 10)

        self.subscriber_img_ = self.create_subscription(
                Image,
                'image_raw',
                self.image_callback,
                10)
        self.subscriber_img_  # prevent unused variable warning
            
            
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        self.get_logger().info('vision_node READY')



    def image_callback(self, imgMsg):
        
        hookDetected = False
        
        img = self.bridge.imgmsg_to_cv2(imgMsg, desired_encoding='passthrough')
        
        
        #lower_red = np.array([0, 0, 50], dtype = "uint8") 

        #upper_red= np.array([50, 50, 255], dtype = "uint8")

        mask = cv2.inRange(img, (0, 0, 50), (50, 50, 255))

        detect = np.sum(mask)

        if detect > 0:
            hookDetected = True
        else:
            hookDetected = False


        # Publication on the /hook topic
        msgHook = Hook()
        msgHook.type = 'detect'
        msgHook.status = hookDetected

        self.publisher_hook_.publish(msgHook)


  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    vision = Vision()

    # Spin the node so the callback function is called.
    rclpy.spin(vision)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vision.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
