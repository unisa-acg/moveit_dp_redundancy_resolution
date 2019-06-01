/*********************************************************************************
 * Copyright (c) 2018, Università degli Studi di Salerno, ALTEC S.p.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/* -------------------------------------------------------------------
 *
 * This module has been developed as part of a collaboration between
 * the Automatic Control Group @ UNISA and ALTEC.
 *
 * Title:	fourr_kazerounian_controller.cpp 
 * Author:	Enrico Ferrentino
 * Org.: 	UNISA - Automatic Control Group
 * Date:    Jul 16, 2018
 *
 * This file implements a ROS node to control the 4R planar
 * manipulator first proposed by Kazerounian & Wang, whose
 * configuration is included in the ROS module
 * fourr_kazerounian_moveit_config. It follows the model of the
 * move_group_interface_tutorial.
 *
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>

#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>
#include <moveit_dp_redundancy_resolution/dp_redundancy_resolution_capability.h>

int main(int argc, char** argv)
{
    // Initializing the node and the move_grou interface

    ros::init(argc, argv, "fourr_kazerounian_controller");
    ros::NodeHandle node_handle;

    /*
     * The async spinner spawns a new thread in charge of calling callbacks
     * when needed. Uncomment the lines below if, as instance, you need to
     * ask for the robot state and expect an answer before the time expires.
     */

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "fourr_planar_arm";
    static const std::string NON_REDUNDANT_PLANNING_GROUP = "q2_q3_q4";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("main", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("main", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Loading the trajectory from the Parameter Server and creating a WorkspaceTrajectory object

    std::string trajectory_name;
    std::vector<double> time;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> roll;
    std::vector<double> pitch;
    std::vector<double> yaw;

    node_handle.getParam("/trajectory/name", trajectory_name);
    node_handle.getParam("/trajectory/time", time);
    node_handle.getParam("/trajectory/x", x);
    node_handle.getParam("/trajectory/y", y);
    node_handle.getParam("/trajectory/z", z);
    node_handle.getParam("/trajectory/roll", roll);
    node_handle.getParam("/trajectory/pitch", pitch);
    node_handle.getParam("/trajectory/yaw", yaw);

    moveit_dp_redundancy_resolution::WorkspaceTrajectory ws_trajectory(trajectory_name, time, x, y, z, roll, pitch, yaw);

    ROS_INFO_NAMED("main", "WorkspaceTrajectory object created with %lu waypoints", time.size());

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.publishPath(ws_trajectory.getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start planning");

    // Planning with dynamic programming redundancy resolution

    ros::ServiceClient dp_redundancy_resolution_service
        = node_handle.serviceClient<moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectory>(move_group::DP_REDUNDANCY_RESOLUTION_SERVICE_NAME);

    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryRequest req;
    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse res;
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;

    ws_trajectory.getWorkspaceTrajectoryMsg(ws_trajectory_msg);

    req.ws_trajectory = ws_trajectory_msg;
    req.planning_group_name = PLANNING_GROUP;
    req.non_redundant_group_name = NON_REDUNDANT_PLANNING_GROUP;
    req.redundancy_parameter_samples = 1440;
    req.redundancy_parameters.push_back("joint1");

    dp_redundancy_resolution_service.call(req, res);

    visual_tools.publishTrajectoryLine(res.solution, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue planning");

    // Planning to a pose goal

    /*
     * Uncomment these instructions to see how the KDL kinematics solver fails in
     * finding a solution for a < 6 DOF manipulator
     */

/*
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 5;
    target_pose1.position.y = 5;
    target_pose1.position.z = 0.4;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("main", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("main", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue planning");
*/

    // Planning to a joint space goal

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("main", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue planning");

	// Planning a joint space path starting from workspace waypoints

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose = ws_trajectory.getWaypoints()[0];
    start_state.setFromIK(joint_model_group, start_pose);
    move_group.setStartState(start_state);

	moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(ws_trajectory.getWaypoints(), eef_step, jump_threshold, trajectory, false);

	ROS_INFO_NAMED("main", "Visualizing plan (%.2f%% achieved)", fraction * 100.0);

	visual_tools.deleteAllMarkers();
	visual_tools.publishPath(ws_trajectory.getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
//	for (std::size_t i = 0; i < time.size(); ++i)
//	    visual_tools.publishAxisLabeled(ws_trajectory.getWaypoints()[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
	visual_tools.trigger();

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to terminate the node");

	ros::shutdown();

	return 0;
}

