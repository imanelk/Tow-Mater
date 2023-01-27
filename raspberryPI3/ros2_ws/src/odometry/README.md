# Odometry node

This node computes the global car speed and parcoured distance. The parcoured distance is used in the "motion planing" node to follow the predefined trajectory. The car speed is not used for now. However, it could be used to regulate the speed of the car more precisely that we did in the "car control" node. 


## How can I start it ? 

``` ros2 run odometry odometry_node ```