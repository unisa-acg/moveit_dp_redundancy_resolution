# panda_moveit_config

This package was initially cloned with
```
git clone https://github.com/ros-planning/panda_moveit_config.git
```
then modified to use the IKFast plugin, configured by following [this tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html).

The modifications listed [here](https://github.com/unisa-acg/fourr_kazerounian_moveit_config/blob/master/Readme.md) have also been implemented, in particular

* the `move_group/MoveGroupDPRedundancyResolutionService` capability has been added to the `move_group.launch` file
* the `dp_redundancy_solver.yaml` has been added to the `config/` folder, together with an `objective_funcion.xml` description
* the `rqt_multiplot.xml` configuration file has been added to the `/config` folder
* the launch file `dp_redundancy_solver.launch` file has been added to the `/launch` folder to load all the parameters needed by the `moveit_dp_redundancy_resolution` library (look [here](https://github.com/unisa-acg/moveit_dp_redundancy_resolution) for additional info). This launch file is called by the `move_group.launch` file.
* the `moveit.rviz` RViz configuration file has been modified to improve the visualization of the robot

## Usage

This package has the same usage as the `fourr_kazerounian_moveit_config` package. Instructions can be found [here](https://github.com/unisa-acg/fourr_kazerounian_moveit_config/blob/master/Readme.md).

As far the non-redundant planning group is concerned, this package does not need to define one, as the IKFast plugin is used, which can handle redundancy by itself.
