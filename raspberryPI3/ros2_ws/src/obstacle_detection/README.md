# Obstacle detection

## Overview

*Obstacle Detection Node* is in charge of *detecting, identificating and locating* 
an obstacle amoung the 6 defined zone.

## Nodes needed

The following node will be use on Obstacle Detection Node:

* Interfaces: 
    * Ultrasonic: to get the distance between the towing car and an object
    * Motors Feedback: it gives motors speed
    * Obstacles: it gives the distance from an object considering an obstacle
    * Fixed Obstacles: it gives the zone number where a fixed obstacle is detected
    * Obstacles ID: it identifies fixed obstacle size and it gives the front zone where it is detected

## Topics

A topic is where is publish/read a message. The topics used are:
* /motors_feedback: it gives the information about motors speed
* /us_data: it gives the distance read by the Ultrasonic Sensors
* /fixed_obstacles: it says when a fixed obstacle is detected
* /obstacles_id: it gives the fied obstcle size and in which zone it is present
 
## Definitions

It will explain the definitions tat are considering in the code.

* Object:
    An object is the presence of something where its distance is more than the threshold distance.

* Obstacle:
    An obstacle is an object where its distance is less or equal to the threshold distance.

* Fixed obstacles:
    An obstacle detected more than 5 seconds in the same zone. 

* Zones:
    There are six zones, each zone is represented by an Ultrasonic Sensor: three in the front 
and three on the back of the car.

## Run test

To be able to run a test about the *detection* and *localisation of fixed obstacles* and , it must follow the next instructions:

1. In a Terminal run the Obstacle Detection Node with the command:
    >> ros2 run obstacle_detection obstacle_detection_node

2. In a second Terminal publish on /motors_feedback topic with the command below 
  with a speed equals to zero:
    >> ros2 topic pub motors_feedback interfaces/msg/MotorsFeedback “l<TAB>”

3. In a third Terminal publish on /us_data topic with the command below with 
  a distance less than the threshold distance:
    >> ros2 topic pub us_data interface/msg/Ultrasonic “f<TAB>”

4. In a forth Terminal display the values published on /fixed_obstacles topic with
  the command below:
    >> ros2 echo /fixed_obstacles

5. In a fifth Terminal: display the values publish on /obstacles_id topic with 
  the command below:
    >> ros2 echo /obstacles_id