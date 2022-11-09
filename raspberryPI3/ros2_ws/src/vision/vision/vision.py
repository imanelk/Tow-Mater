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

class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        elif len(approx) == 6:
            shape = "hexagon"
        elif len(approx) == 10 or len(approx) == 12:
            shape = "star"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
        # return the name of the shape
        return shape    

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
        img = self.bridge.imgmsg_to_cv2(imgMsg, desired_encoding='passthrough')

        
        ## Images processing 
        resized = imutils.resize(image, width=300) #redimensionnement
        ratio = image.shape[0] / float(resized.shape[0])
        
        # convert the resized image to blur it slightly,
        # and threshold it

        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        
        # find contours in the thresholded image and initialize the
        # shape detector
        # get all the possible shapes
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        

        cnts = imutils.grab_contours(cnts)
        sd = ShapeDetector()

        # loop over the contours
        for c in cnts: 
            #detect shape from contour
            shape = sd.detect(c)
            
            if (shape == "star"): 
                hookDetected = True
                self.get_logger().info('Hook detected')
            else: hookDetected = False
            # resize the contour
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")


        # Publication on the /hook topic
        msgHook = Hook()
        msgHook.type = 'Detect'
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