# Side detection

This package is used to check if there is a tall and large obstacle (like a wall) and if it is on the left or the right of the gei car. It is used when we detect an obstacle in front of the car and we want to avoid it. To to that, it collects the data coming from the rplidar package (topic /scan) and analyse the front left and right zones. 

## Initialisation of the LiDAR

The main part of the initialisation has been made in the rplidar package. However, it is necessary put the Lidar in the correct direction before starting the package. See the pdf "Lidar_init_and_ranges.pdf".

## Analysed zones

Here, we are focused on detecting an obstacle that could influence the avoidance trajectory. Thus, it has to be made by analysing the front zones. See the pdf "Lidar_init_and_ranges.pdf".

## How does it work ? 

The idea is to return a message which informs if this obstacle is more on the left or on the right by comparing the means of the distances with obstacles. We also add the minimum distance with an obstacle in the two zones. Thus, we are able to :
- see if it is better to avoid an obstacle by the left or the right,
- see if there is enough space between the car and the large obstacle. 

## How can I start it ?

``` ros2 run side_detection side_detection_node ```