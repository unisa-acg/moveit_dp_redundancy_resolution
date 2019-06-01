# moveit_dp_redundancy_resolution

This package collects all the libraries necessary to implement the redundancy resolution through dynamic programming optimization. It depends on the `moveit_dp_redundancy_resolution_msgs` package for messages and services definitions.

Like all the other move_group capabilities, the `dp_redundancy_resolution_capability` is a plugin that can be loaded at runtime. Its service can be called by any implementation of the `MoveGroupInterface`, such as the `fourr_kazerounian_controller_node` (see the `fourr_kazerounian_controller` package).

## Expected parameters

The `moveit_dp_redundancy_resolution` library expects a number of parameters loaded onto the parameter server. Parameters can be loaded by configuring the launch file(s) for each specific robot. Examples are provided in the `panda_moveit_config` and `fourr_kazerounian_moveit_config` packages. See, in particular `config/dp_redundancy_solver.yaml` and `launch/dp_redundancy_solver.launch` of both packages.

* The `DynamicProgrammingSolver` class uses the `ObjectiveFunction` class to enable the support of arbitrary objective functions. The user defines an objective function XML description file, whose complete filename (including its path) is loaded on the parameter server under the name
```
/move_group/dp_redundancy_solver_config/objective_function
```
See the package `fourr_kazerounian_moveit_config` for an example of objective function description.
* The `moveit_dp_redundancy_resolution` package generates several files concerning the generated globally-optimal solution: grids (see [Grids format](#grids-format)), colormaps (needed for a quick-look visualization of the grid generation results), bagfiles (containing the joint space globally-optimal solution). Files are generated in the folder indicated by the parameter
```
/move_group/dp_redundancy_solver_config/output_path
```
that must be loaded on the parameter server. Make sure to configure your launch file the right way!
* In order for the `StateSpaceGrid2D` class to know the number of IK solutions to look for, the robot type (i.e. planar, spherical, regional, spatial) must be specified on the parameter server. Make sure to load the parameter named
```
/move_group/dp_redundancy_solver_config/robot_type
```

## Remarks for users of the package

* The `WorkspaceTrajectory` class differs from the `WorkspaceTrajectory` message in that the former provides greater functionality than the latter, e.g. the capability of creating a trajectory in the task space corresponding to a given joint space trajectory.
* At the moment the state_space_grid library only has classes to support 2D grids (i.e. one redundancy parameter), but shall be extended in the future to be able to deal with, at least, two redundancy parameters. The `GetOptimizedJointsTrajectory` service definition already provides the support for more than one redundancy parameter, but requests of this type will be rejected by the current implementation of the state_space_grid library.
* An `ObjectiveFunction` is made up of one or more performance indices, whose weights must sum to one. At the moment, only two performance indices are implemented: square norm of joint velocities and distance between a robot-fixed reference frame and an obstacle. More performance indices can be defined by implementing the abstract class `PerformanceIndex`.

## Known issues

* The error

```
rviz: /build/ogre-1.9-mqY1wq/ogre-1.9-1.9.0+dfsg1/OgreMain/include/OgreAxisAlignedBox.h:252: void Ogre::AxisAlignedBox::setExtents(const Ogre::Vector3&, const Ogre::Vector3&): Assertion `(min.x <= max.x && min.y <= max.y && min.z <= max.z) && "The minimum corner of the box must be less than or equal to maximum corner"' failed.
```

may happen under certain circumstances. It is a known bug, Google it for additional info. Apparently, it arises under very different circumstances, sometimes unpredictably. In the case of this package, it is sufficient to swap the functions `exportToBinary` and `exportToMaps` in `computeOptimalSolutionSpatial_` or just remove the `exportToMaps` function. In order for the issue to arise, make sure the `/grids` folder in the `<robot_name>_moveit_config` package is empty!

## Grids format

The `StateSpaceGrid2D` and `StateSpaceMultigrid2D` classes are able to generate state space grids and export them to binary files in a custom format, here described. Each field written in the binary file is here reported on a different row, but no new lines or carriege return bytes are inserted in the file, meaning that the first byte of the second row is the one following the last byte of the first row and so on.

|Byte index                           |C++ type        |Number of bytes | Semantics                                                                           |
|-------------------------------------|----------------|---------------:|-------------------------------------------------------------------------------------|
|0                                    |unsigned long   |8               |Number of waypoints, i.e. x-axis of the grid (or column index)                       |
|4                                    |unsigned long   |8               |Number of samples of the redundancy parameter, i.e. y-axis of the grid (or row index)|
|8                                    |char            |1               |NaN (n) or double (d) indicator. If 'n', another indicator follows.                  |
|...                                  |...             |...             |...                                                                                  |
|i                                    |char            |1               |NaN (n) or double (d) indicator. Assume this is 'd'.                                 |
|i+1                                  |double          |8               |First joint position                                                                 |
|i+9                                  |double          |8               |Second joint position                                                                |
|...                                  |...             |...             |...                                                                                  |
|i+1+8(num_joints-1)                  |double          |8               |Last joint position                                                                  |
|i+1+8(num_joints)                    |char            |1               |NaN (n) or double (d) indicator. Assume this is 'n'.                                 |
|i+2+8(num_joints)                    |char            |1               |NaN (n) or double (d) indicator. Assume this is 'd'.                                 |
|i+3+8(num_joints)                    |double          |8               |First joint position                                                                 |
|...                                  |...             |...             |...                                                                                  |
|i+3+8(num_joints)+8(num_joints-1)    |double          |8               |Last joint position                                                                  |
|...                                  |...             |...             |...                                                                                  |

The first two elements in the file are the number of waypoints and the number of the redundancy parameter samples. The matrix of data is then serialized in the order rows-columns, meaning that a row is written entirely, i.e. all redundancy parameter values for a waypoint, before writing the next value in the following row, i.e. the next waypoint. NaNs are tracked with a single byte equivalent to the character 'n', while valid position vectors are preceeded by a single byte equivalent to the character 'd'.

## Contributing to the development

Here are some guidelines to contribute to the development of the moveit_dp_redundancy_resolution package

### Commit messages

When writing commit messages, please use the following conventions

* ADD adding new feature
* FIX a bug
* DOC documentation only
* REF refactoring that doesn't include any changes in features 
* FMT formatting only (spacing...)
* MAK repository related changes (e.g., changes in the ignore list)
* TST related to test code only

Use bullet lists for commits including more than one change. See the latest commit messages for an example before making your first commit!
