# multi_robot_coverage_path_planning_ros
A ROS-based project implementing multi-robot coverage path planning in a Stage environment. The robots navigate through obstacles using motion planning, communicate their movements via custom messages, and reset to initial positions. The project includes Python nodes for each step and utilizes ROS Stage for execution.

## Overview

This project implements multi-robot coverage path planning in the ROS (Robot Operating System) environment using Stage. The robots navigate through obstacles, communicate their movements, and reset to initial positions.

## Features

- **Motion Planning:** Robots follow a specified motion pattern, avoiding obstacles using sensors.
- **Communication Between Robots:** Robots share information about the cells they visit using a custom "navigationdata" message type.
- **Cell Counting:** Each robot autonomously calculates the number of cells visited and the ID of the last cell.
- **Resetting Robot Positions:** A Python node resets the robot positions using the `/reset_positions` service of ROS Stage.

![image](https://github.com/hsiangenlinlin/multi_robot_coverage_path_planning_ros/assets/125904538/78602c8c-0672-4e7f-ae77-04496660c783)

## Execution

1. Ensure ROS and Stage are properly installed.
2. Run the nodes using `rosrun` or `roslaunch` commands.
3. Visualize the execution in ROS Stage.
