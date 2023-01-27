# Map Navigation

This node was created to get the GPS coordinates of the damaged and compute the route to reach it. For now, it only collects the GPS coordinates and publish them on a topic. It also prints a message of acknowledgment with the coordinates to inform the user that the coordinates have been received. 

## How can I start it ? 

On the damaged car :
``` ros2 run can can_rx_node ``` (run the can node to get the GPS coordinates and sends them )

On the towing car : 
``` ros2 run map_navigation map_navigation_node ```