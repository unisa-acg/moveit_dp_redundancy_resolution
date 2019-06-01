# panda_controller

## Introduction

The `panda_controller` is the package launching the ROS node where the workspace trajectory is defined and the request is sent to the move_group node for optimal redundancy resolution through dynamic programming. It can generate three different trajectories or load a trajectory from file (the behavior can be changed by modifying the `trajectory.yaml` configuration file in the `config/` folder.

## Remarks

* Do not try to use the `setMaxVelocityScalingFactor` and `setMaxAccelerationScalingFactor` functions to scale down the trajectory velocity when not using a proper planner. Remember that the `computeCartesianPath` service is not exactly a planner and, thus, does not use such scaling factors. Using them will not affect the final result. Look at [this thread](https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/) for additional info.
* At the moment, the `panda_controller` is only able to save the `square_trajectory.traj` trajectory file and load a file with the same name. For this reason, when loading a trajectory, make sure it is named this way. Also be aware that executing the node with the `square_trajectory` setting will overwrite this file.
* In this package, together with the `square_trajectory.traj` trajectory, a `square_trajectory_with_time_jump.traj` file is also included which contains a trajectory for which timestamps have not been evenly-spaced. It must only be used as a demonstration of how a trajectory looks like when the time parametrization is not changed after the planning performed by the `computeCartesianPath` function.
* Look at the comments in the source file for additional info.
