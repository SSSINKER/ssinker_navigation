# ssinker_navigation

ROS navigation stack configuration package for KABOAT 2021 mission 3. </br>

Execution Guide
--
* ```Config.launch```
  * Launch all sensors.</br>
  *Recommended: do not use this launch file. use hopping_tour/sensors.launch instead.*
* ```navigation.launch```
  * Launch ```move_base``` topic with given configuration parameters in ```/param``` folder.

* ```pointfollower.cpp```
  * resembles ```hopping_tour_IMU``` of hopping_tour package.