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
        det = cv2.QRCodeDetector()
        info, box_coordinates, _ = det.detectAndDecode(img)

        if box_coordinates is None:
            # print('No Code')
            hookDetected = False
        else:
            # print("Côté détecté = ", info)
            hookDetected = True

        # tracer les contours du qr code sur l'image
#        if box_coordinates is not None:
#            box_coordinates = [box_coordinates[0].astype(int)]
#            n = len(box_coordinates[0])
#            for i in range(n):
#                cv2.line(img, tuple(box_coordinates[0][i]), tuple(box_coordinates[0][(i+1) % n]), (0,255,0), 3)

        # x_coordinates = [(box_coordinates[0][0][0]) + (((box_coordinates[0][1][0])-(box_coordinates[0][0][0]))/2)]
        # y_coordinates = [(box_coordinates[0][0][1]) + (((box_coordinates[0][2][1])-(box_coordinates[0][0][1]))/2)]
        # middle_coordinates = [x_coordinates, y_coordinates]
        # width_qr = box_coordinates[0][1][0]-box_coordinates[0][0][0]

        # print("box coordinates = ", box_coordinates[0])
        # print(" ")
        # print("middle coordinates = ", middle_coordinates)
        # print(" ")
        # print("width of qr  = ", width_qr)
 
        #cv2.imshow('Output', img) #print l'image avec le QRcode encadré
        msgHook=Hook()
        msgHook.type = 'Detect'
        msgHook.status = hookDetected

        self.publisher_hook_.publish(msgHook)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()

  
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