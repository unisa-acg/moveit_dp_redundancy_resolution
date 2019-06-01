# ROS/MoveIt! extension for redundancy resolution with dynamic programming

[![DOI](https://zenodo.org/badge/189755571.svg)](https://zenodo.org/badge/latestdoi/189755571)

This repository is a collection of ROS packages needed to perform globally-optimal redundancy resolution for kinematically redundant robotic systems.

The `moveit_dp_redundancy_resolution` and `moveit_dp_redundancy_resolution_msgs` packages contain the core libraries, while all the `fourr_kazerounian_*` and `panda_*` packages are support packages to run demos on a 4R planar robot and 7R spatial robot respectively.

Refer to the Readmes of the single packages for further information. It is convenient to explore the documentation in the following order:

- [`moveit_dp_redundancy_resolution`](./moveit_dp_redundancy_resolution/Readme.md)
- [`moveit_dp_redundancy_resolution_msgs`](./moveit_dp_redundancy_resolution_msgs/Readme.md)
- [`fourr_kazerounian_moveit_config`](./fourr_kazerounian_moveit_config/Readme.md)
- [`panda_moveit_config`](./panda_moveit_config/Readme.md)
- [`fourr_kazerounian_controller`](./fourr_kazerounian_controller/Readme.md)
- [`panda_controller`](./panda_controller/Readme.md)
- [`fourr_kazerounian_description`](fourr_kazerounian_description/Readme.md)
- [`panda_ikfast_panda_arm_plugin`](./panda_ikfast_panda_arm_plugin/Readme.md)
