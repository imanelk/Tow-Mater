# Obstacle Detection

The node is detecting obstacles that are in front of and behind the car. It uses the six Ultrasonic Sensors only to describe six areas where the obstacles are : 
- front-left
- front-middle
- front-right
- back-left
- back-middle
- back-right
It is also capable to identify when a obstacle is fixed : if the obstacle is still there after 5 seconds, then it is fixed. 

## What are the published information ? 

The node publishes on three topics : "obstacles_id", "obstacle" and "fixed_obstacles". Here is a little explaination of what these messages are containing and what for they are used. 

*Obstacle* 
This message is composed of the distance with an obstacle that the six Ultrasonic Sensors are detecting. So it gives a general information of where the obstacles are.

*Fixed Obstacles*
This message is set to True when an obstacle is still detected 5 seconds after its first detection. So, it means that it is fixed. Otherwise, the message it False.

*Obstacles ID*
This message is checked by "motion planning" when the obstacle is fixed in front of the car and we want to avoid it. It gives the following information : 
- is the obstacle big ? (so the three front Ultrasonic Sensors are detecting an obstacle at a distance of less than 1 meter )
- where is the obstacle ? which sensors are activated ? 

## How ca I start it ? 

``` ros2 run obstacle_detection obstacle_detection_node ```