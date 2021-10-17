# ssinker_navigation

ROS navigation stack configuration package for KABOAT 2021 mission 3. </br>

### Recent Changes
* 2021.10.17 </br>
added pointfollower.cpp file that publish overriding ```cmd_vel``` topic of base_local_planner node.

Launch Files
--
## Config.launch
  * Launch all sensors.</br>
  *Recommended: do not use this launch file. use hopping_tour/sensors.launch instead.*
## navigation.launch
  * Launch ```move_base``` topic with given configuration parameters in ```/param``` folder.

Nodes
--
## pointfollower
resembles ```hopping_tour_IMU``` of hopping_tour package.
 * Subscribed Topics
   * ```/global_plan``` (nav_msgs/Path) : Planned path to follow. Commonly generated from base_local_planner node of ROS navigation package.
 * Published Topics
   * ```/cmd_vel``` (geometry_msgs/Twist) : Command velocity. 
     * linear: (vx, 0, 0)
     * angular: (0, 0, wz)
 * Parameters
   * NaN
 * Required tf Transforms
   * ```map``` -> ```odom```: Commonly generated from SLAM node.
