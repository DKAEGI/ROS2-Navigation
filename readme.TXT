ROS2 Navigation Project to get familiar with the ROS2 SLAM and Nav Stack Tools

1. project_mapping: 
Creating a map with the cartographer and providing the map to other nodes. 

2. project_localization: 
Localize the robot with the amcl_node. 
Creating a service to save amcl_pose position and orientation of different spots. 
Custom service message in project_custom_interfaces.

3. project_path_planning: 
Adding the necessary parameter files in order to properly configure the controller_server, planner_server, recoveries_server and bt_navigator nodes. 
Adding global and local costmap to it. 
Creating a file to use different /initialpose and saving these spot parameters in a yaml-file. 
Setting up the path planning in order to navigate to a desired goal position

Proof of concept succesfully tested on Turtlebot. 
Parameter files can be further optimized to get better results. 
			  
