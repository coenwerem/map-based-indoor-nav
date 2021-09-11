# TEAM Kognitive Robotics: 2021 PARC Engineers League (Phase 1, Task 3 Solution) 

## Introduction

Phase 1 of the 2021 PARC Engineer's League competition entailed developing robot software for the safe navigation of a delivery robot, from pick-up to customer package drop-off within a simulated environment. This phase further involved validating developed software through appropriate simulations and visualizations on Gazebo and RViz respectively, to satisfy competition requirements and achieve desirable scores on task metrics.<br>
   
As a member of Team Kognitive Robotics, I was tasked with the third task of Phase 1, which was to write robot software that allowed for safe maneuvers of the robot (with dynamic obstacle avoidance) from a random initial pose in the world to a marked package dropoff location in simulation. 

## Dependencies

**Dependencies used for the above-described part of the project were:**

*  `geometry_msgs`
*  `roscpp`
*  `rospy`
*  `std_msgs`
*  `gazebo_msgs`
*  `geometry_msgs`
*  `gazebo_msgs`
*  `move_base_msgs`
*  `sensor_msgs`
*  `nav_msgs`


**To run this package on your machine, you will need the following ROS packages. Ideally, you should have a ROS Melodic installation on a machine running Ubuntu 18.04:**


* `slam-gmapping`: ROS package for Simultaneous Localization & Mapping (SLAM).
    * `$ sudo apt-get install ros-melodic-slam-gmapping`

* `navfn`: ROS package for planning global paths to the navigation goal
    * `$ sudo apt-get install ros-melodic-navfn`

* `global-planner`: ROS package for path planning using A*
    * `$ sudo apt-get install ros-melodic-global-planner`

<br>

## Deployed Approach

For this phase of the competiton, I employed the ROS Autonomous Navigation stack by first building a static map of the environment through teleoperation and using the slam-gmapping package for SLAM. <br>

Next, I applied the Adaptive Monte-Carlo Localization (AMCL) technique (for global localization) alongside the ROS Trajectory Planner (for motion planning) to plan an optimal navigation path by creating appropriate configuration files (*_params.yaml files) in the ./config folder and important arguments and parameters (e.g initial robot pose, map file, map server, etc.) in the task solution launch file. I also iteratively tuned several parameters notably the robot's footprint, inflation layer radius, and controller obstacle avoidance weight to arrive at an optimal and obstacle-free path from the robot to the goal. <br>

Finally, to make for a dynamic solution, I wrote a Python script to programmatically obtain the pose of the goal location (marker) within the Gazebo environment and define the 2D navigation goal (the retrieved pose of the marker) by subscribing and publishing to appropriate ROS topics.<br>

The command required to run this package is as follows: <br>
` roslaunch task3_solution_pkg task3_solution.launch ` <br>


A short gif of the code in action is shown below:<br>

![Task 3 Solution](./resources/task3_sol_vid.gif)



You can also find a longer YouTube video showing the above solution [here](https://youtu.be/pzsVFjBKmpc).<br>



## Challenges Faced

* The robot always appeared at the wrong pose relative to the map in RViz. I fixed this by setting initial pose AMCL arguments.
* The path was always planned dangerously close to obstacles causing the robot to bump into them. I solved this by increasing the inflation radius in the costmap_common_params.yaml file.
* The occassional `catkin_make` fails.
* The height of the RP LiDAR from the ground was lower than some obstacles (e.g. the base of the gazebo model in task3.world). This led to some undefined areas in the built static map and hence, poorly planned paths. However, I tuned the ROS Trajectory planner parameters to account for this constraint.
* In sum, Gazebo is a memory and battery hog. Having to code, run simulations, and debug with a laggy mid-range PC all while trying to economize battery power was frustrating to say the least. Other physics engines and robot simulators like Drake and CopelliaSim Edu (formerly VREP) are free, work just as well as Gazebo, and integrate smoothly with ROS, without the aforementioned constraints.
