# Obstacle Avoidance

This package collects the information sent by the Ultrasonic sensors and the Lidar and chooses the best avoidance trajectory. See pdf "Avoidance_choices.pdf".


## How does it work ? 

The idea is to get the information from the "obstacle detection" (in the Raspberry) and "side detection" (in the Jetson) packages. First, "obstacle detection" gives us information about an obstacle which is in front of the car. Secondly, "side detection" gives us information about an obstacle which is in the front left and right zones. Then, we can know if the obstacle that we have to avoid is on the left or the right and if we have enough space to avoid it.


## How can I start it ?

``` ros2 run obstacle_avoidance obstacle_avoidance_node ```