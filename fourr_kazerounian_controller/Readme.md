# fourr_kazerounian_controller

This repository collects the source files and documentation of the move_group interface used to control the move_group node implementing the 4R planar manipulator from [Kazerounian & Wang](https://journals.sagepub.com/doi/10.1177/027836498800700501).

The MoveGroup class exposes a set of default capabilities that can be used for planning and other needs, that are:

* `MoveGroupCartesianPathService`: Computing straight line Cartesian paths with collision checking via a ROS service
* `MoveGroupExecuteService`: Allow execution of previously computed paths via a ROS service
* `MoveGroupExecuteTrajectoryAction`: Allow execution of previously computed trajectories via a ROS action
* `MoveGroupKinematicsService`: Compute forward and inverse kinematics via ROS services
* `MoveGroupMoveAction`: Compute motion plans via a ROS action
* `MoveGroupPlanService`: Compute motion plans via a ROS service
* `MoveGroupQueryPlannersService`: Allow querying of available planners (loaded from the motion planning plugin) via a ROS service
* `MoveGroupStateValidationService`: Provide a ROS service that allows for testing state validity
* `MoveGroupGetPlanningSceneService`: Provide a ROS service that allows for querying the planning scene
* `ApplyPlanningSceneService`: Provide a ROS service for blocking updates to the planning scene
* `ClearOctomapService`: Provide a ROS service that allows for clearing the octomap

Be aware that the capabilities above are not immediately visible through the `MoveGroupInterface` implemented in this package, but they are called by the move_group node executing in the scope of the controlled package.

`MoveGroupCartesianPathService` and `MoveGroupMoveAction` capabilities are those mainly used for planning in the `move_group_interface_tutorial`. Both of them are also used in this package for demonstration, together with the `MoveGroupDPRedundancyResolutionService` developed in this project.

## Planning schemes used in this package

* Planning with globally-optimal redundancy resolution, by using the plugin developed in this project, using dynamic programming.
* Planning to a pose goal (commented out): a goal is set in the workspace and a plan is asked to OMPL by using the KDL kinematic solver. The planner fails in finding the trajectory as KDL has poor performances with <6 DOF systems. This scheme exploits the `MoveGroupMoveAction` capability.
* Planning to a joint space goal: a goal in the joint space is set and a plan is found by using the `MoveGroupMoveAction` (masked by the plan function in the code).
* Planning a cartesian path: a (fine) list of waypoints is loaded from the Parameter Server with timing information in order to create a `WorkspaceTrajectory`. The waypoints describe a circumference in the xy plane. These are passed to the `MoveGroupCartesianPathService` to compute a finer set of control points that are kinematically inverted to find a continuous trajectory in the joint space. Remember that step resolution must be smaller than the maximum distance between waypoints, otherwise the trajectory cannot be achieved.

## Available trajectories

These trajectories can be configured in the package launch file.

* `ws_trajectory_matlab.yaml`: this is the trajectory exported from the MATLAB experiments. It is the one used with the optimal redundancy resolution and cannot be used for computing cartesian path through the default `MoveGroupCartesianPathService` capability because of the overlap between waypoints
* `ws_trajectory.yaml`: this is a trajectory with meaningless timestamps, that can be used for computing cartesian paths through the `MoveGroupCartesianPathService` capability
* `ws_trajectory_yaw_0.yaml`: same as above but with yaw=0.
