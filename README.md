# TEAM Kognitive Robotics: 2021 PARC Engineers League (Phase 1, Task 3 Solution) 

## Introduction

This project is my solution for the third and final task of Phase 1 of the 2021 Pan-African Robotics Competiton (PARC) Engineers' League, where my team also placed in the finals. The competition entailed developing robot software for the safe navigation of a delivery robot &mdash; from pick-up to customer package drop-off &mdash; within a custom Gazebo environment. We were also required to validate our software to meet specific goals and task completion requirements.<br>
   
As a member of Team Kognitive Robotics, I wrote robot software to enable the delivery robot execute safe maneuvers (with dynamic obstacle avoidance) from a random initial pose in the world to a marked package dropoff location. 

## Dependencies

**The dependencies used for the above-described part of the project were:**

*  `roscpp`
*  `rospy`
*  `std_msgs`
*  `gazebo_msgs`
*  `geometry_msgs`
*  `move_base_msgs`
*  `sensor_msgs`
*  `nav_msgs`


**To run this package, you will need the following ROS packages. Ideally, you should have ROS Melodic installed on an Ubuntu 18.04 machine:**


* `slam-gmapping`: ROS package for Simultaneous Localization & Mapping (SLAM).
    * `$ sudo apt-get install ros-melodic-slam-gmapping`

* `navfn`: ROS package for planning global paths to the navigation goal
    * `$ sudo apt-get install ros-melodic-navfn`

* `global-planner`: ROS package for path planning using A*
    * `$ sudo apt-get install ros-melodic-global-planner`

<br>

## Deployed Approach

For this phase of the competiton, I employed the ROS Autonomous Navigation stack by first building a static map of the environment through teleoperation, and then using the slam-gmapping package for SLAM. <br>

Next, I applied the Adaptive Monte-Carlo Localization (AMCL) technique (for global localization) alongside the ROS Trajectory Planner (for motion planning) to plan an optimal navigation path by creating appropriate configuration files (*_params.yaml files) in the ./config folder and important arguments and parameters (e.g initial robot pose, map file, map server, etc.) in the task solution launch file. I also iteratively tuned several parameters notably the robot's footprint, inflation layer radius, and controller obstacle avoidance weight to arrive at an optimal and obstacle-free path from the robot to the goal. <br>

Finally, to make for a dynamic solution, I wrote a Python script to programmatically obtain the pose of the goal location (marker) within the Gazebo environment and define the 2D navigation goal (the retrieved pose of the marker) by subscribing and publishing to appropriate ROS topics.<br>

The command required to run this package is as follows: <br>
` roslaunch task3_solution_pkg task3_solution.launch ` <br>


A short gif of the code in action is shown below:<br>

![Task 3 Solution](./resources/task3_sol_vid.gif)
