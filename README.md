# Home_Service_Robot


In this project, I have used almost everything that I had learned in this Nanodegree Program to build a Home Service Robot in ROS.

# Project Goal


The project goal is to simulate a full home service robot capable of navigating to pick up and deliver virtual objects.

# Decscription


#### The project consist of following main parts:

 1. A Gazebo world and a mobile robot.
 2. The ROS packages: gmapping, turtlebot_teleop, turtlebot_rviz_launchers, and turtlebot_gazebo.
 
#### Directories Created

 1. map: Inside this directory, you will store your gazebo world file and the map generated from SLAM.
 2. scripts: Inside this directory, you’ll store your shell scripts.
 3. pick_objects: You will write a node that commands your robot to drive to the pickup and drop off zones.
 4. add_markers: You will write a node that model the object with a marker in rviz.  
    
## How it works

The mobile robot first drives around the house and scan it using lidar laser scanner for generating a static map about this place by turning the lidar data into depth camera data.
Having the map, it uses odometry and laser data to localize itself with adaptive monte carlo localization (AMCL) using amcl_demo.launch ros file. 
Upon receiving a navigation goal, it plans forward the trajectory using Dijkstra's algorithm, a kind of uniform cost search path planning algorithm and navigate to the goal.

### Some issues

I found some minor issues right now, which I will try to improve in future:
 1. Sometimes the robot is rotating too much. Maybe due to obstacle.
 2. After picking the virtual object it is still moving some distance.

## Conclusion

In this final project, following knowledge is combinded:

 1. Fine-tuning the localizaton algorithm.
 2. Writing the ROS nodes to publish and subscribe to the required topic.
 3. Apply and tuning the mapping and trajectory planning algorithm.
