# moveit_dp_redundancy_resolution_msgs

This repository includes the messages and services definitions needed for the MoveGroupDPRedundancyResolutionService, extending the move_group capabilities.

For the time being it defines:

* WorkspaceTrajectory.msg: message corresponding to the WorkspaceTrajectory class in workspace_trajectory.h, defined in the moveit_dp_redundancy_resolution package. It is simply made by a vector of waypoints with relating timestamps.
* GetOptimizedJointsTrajectory.srv: the request/response definition for the MoveGroupDPRedundancyResolutionService
