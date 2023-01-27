# Vision

The node vision is a node supervising the detection of a QR code by the camera. It was coded using Python because we needed the Python packages :
- **CvBridge** that converts images between ROS and OpenCV
- **OpenCV** that helps for image processing

We use this part of the code to identify the broken car from a simple obstacle: whenever the car detects an obstacle, it researches a particular QR code. If this particular QR code is found, then the towing car recognizes the damaged car and can reverse towards it in order to tow it.

## Usage
This node should take in pictures from the camera. It should detect any QR code on the picture, then read them and tell if the code we are looking for has been found. The code should research for codes containing the words "avant" or "arrière".

When those codes have been detected, the message that we send back contains the status of the QR code (detected or not), and its coordinates on the picture (that we computed when it was detected).


## Roadmap

For the future, it could be interesting to use the width of the QR code to be able to identify the distance from the camera to the QR code.

## How ca I start it ? 

``` ros2 run vision vision_node ```