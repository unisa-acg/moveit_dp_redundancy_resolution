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
 * Title:   panda_controller.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Sep 6, 2018
 *
 * This file implements a ROS node to control the Panda manipulator
 * from Franka Emika, whose configuration is included in the ROS module
 * panda_moveit_config. It follows the model of the
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

static const std::string POSE_GOAL_TRAJECTORY       = "pose_goal_trajectory";
static const std::string CARTESIAN_TRAJECTORY_LONG  = "cartesian_trajectory_long";
static const std::string SQUARE_TRAJECTORY          = "square_trajectory";
static const std::string LOADED_TRAJECTORY          = "loaded_trajectory";

static const std::map<const std::string, const int> trajectory_map =
        {{POSE_GOAL_TRAJECTORY,     0},
        {CARTESIAN_TRAJECTORY_LONG, 1},
        {SQUARE_TRAJECTORY,         2},
        {LOADED_TRAJECTORY,         3}};

int main(int argc, char** argv)
{
    // Initializing the node and the move_group interface

    ros::init(argc, argv, "panda_controller");
    ros::NodeHandle node_handle;

    /*
     * The async spinner spawns a new thread in charge of calling callbacks
     * when needed. Uncomment the lines below if, as instance, you need to
     * ask for the robot state and expect an answer before the time expires.
     */

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("main", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("main", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    std::string trajectory_name;
    node_handle.getParam("/trajectory_name", trajectory_name);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    moveit_dp_redundancy_resolution::WorkspaceTrajectory * ws_trajectory = NULL;

    switch(trajectory_map.at(trajectory_name))
    {
    case 0:
    {
        // We plan to a pose goal in order to generate a trajectory connecting
        // the current state to the pose goal.

        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = -0.6;
        target_pose1.position.y = -0.4;
        target_pose1.position.z = 0.7;
        move_group.setPoseTarget(target_pose1);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group.clearPoseTargets();

        // The planning system only returns a trajectory in the joint space,
        // we need to create a trajectory from it by using FW kinematics.

        try
        {
            ws_trajectory = new moveit_dp_redundancy_resolution::WorkspaceTrajectory(
                    POSE_GOAL_TRAJECTORY,
                    plan.trajectory_,
                    move_group.getCurrentState(),
                    PLANNING_GROUP);

        }catch(ros::Exception & e)
        {
            ROS_ERROR_NAMED("main", "%s", e.what());
            ros::shutdown();
            exit(1);
        }

        // We visualize the path by using the computed trajectory (task space trajectory)
        // and not the one from the plan (joint space trajectory), so that we can verify
        // that the robot moves along the trajectory.

        ROS_INFO_NAMED("main", "Visualizing plan as trajectory line");

        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose1, "pose1");
        visual_tools.publishPath(ws_trajectory->getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
        visual_tools.trigger();

        break;
    }
    case 1:
    {
        // We plan a Cartesian path by giving some waypoints, then ask the planner to interpolate
        // between them to have a higher number of waypoints that we can use for planning

        std::vector<geometry_msgs::Pose> waypoints;

        geometry_msgs::Pose start_pose;
        start_pose.orientation.w = 1.0;
        start_pose.position.x = 0.5;
        start_pose.position.y = 0;
        start_pose.position.z = 0.8;

        robot_state::RobotState start_state(*move_group.getCurrentState());
        bool success = start_state.setFromIK(joint_model_group, start_pose);
        move_group.setStartState(start_state);

        if(!success)
        {
            ROS_ERROR("Initial pose could not be inverted through inverse kinematics");
            ros::shutdown();
            exit(1);
        }

        start_pose.position.z -= 0.6;
        waypoints.push_back(start_pose);

        start_pose.position.y -= 0.5;
        waypoints.push_back(start_pose);

        start_pose.position.x -= 1.0;
        waypoints.push_back(start_pose);

        start_pose.position.x += 0.4;
        start_pose.position.y += 0.4;
        start_pose.position.z += 0.9;
        waypoints.push_back(start_pose);

        start_pose.position.y += 0.4;
        waypoints.push_back(start_pose);

        start_pose.position.z -= 0.9;
        start_pose.position.x -= 0.5;
        waypoints.push_back(start_pose);

        // We want the Cartesian path to be interpolated at a resolution of 1 cm
        // which is why we will specify 0.01 as the max step in Cartesian
        // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
        // Warning - disabling the jump threshold while operating real hardware can cause
        // large unpredictable motions of redundant joints and could be a safety issue
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        try{

            ws_trajectory = new moveit_dp_redundancy_resolution::WorkspaceTrajectory(
                    CARTESIAN_TRAJECTORY_LONG,
                    trajectory,
                    move_group.getCurrentState(),
                    PLANNING_GROUP);

        }catch(ros::Exception & e)
        {
            ROS_ERROR_NAMED("main", "%s", e.what());
            ros::shutdown();
            exit(1);
        }

        ROS_INFO_NAMED("main", "Visualizing Cartesian path (%.2f%% achieved) as trajectory line", fraction * 100.0);

        visual_tools.deleteAllMarkers();
        visual_tools.publishPath(ws_trajectory->getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
        visual_tools.trigger();

        break;
    }
    case 2:
    {
        std::vector<geometry_msgs::Pose> waypoints;

        tf::Quaternion q;
        geometry_msgs::Pose start_pose;

        q = tf::createQuaternionFromRPY(0, -M_PI/2, 0);
        start_pose.orientation.x = q.x();
        start_pose.orientation.y = q.y();
        start_pose.orientation.z = q.z();
        start_pose.orientation.w = q.w();
        start_pose.position.x = 0.3;
        start_pose.position.y = -0.3;
        start_pose.position.z = 0.8;

        robot_state::RobotState start_state(*move_group.getCurrentState());
        bool success = start_state.setFromIK(joint_model_group, start_pose);
        move_group.setStartState(start_state);

        if(!success)
        {
            ROS_ERROR("Initial pose could not be inverted through inverse kinematics");
            ros::shutdown();
            exit(1);
        }

        start_pose.position.z -= 0.4;
        waypoints.push_back(start_pose);

        q = tf::createQuaternionFromRPY(0, -M_PI, 0);
        start_pose.orientation.x = q.x();
        start_pose.orientation.y = q.y();
        start_pose.orientation.z = q.z();
        start_pose.orientation.w = q.w();
        start_pose.position.y += 0.6;
        waypoints.push_back(start_pose);

        q = tf::createQuaternionFromRPY(0, +M_PI/2, 0);
        start_pose.orientation.x = q.x();
        start_pose.orientation.y = q.y();
        start_pose.orientation.z = q.z();
        start_pose.orientation.w = q.w();
        start_pose.position.z += 0.4;
        waypoints.push_back(start_pose);

        q = tf::createQuaternionFromRPY(M_PI, M_PI, 0);
        start_pose.orientation.x = q.x();
        start_pose.orientation.y = q.y();
        start_pose.orientation.z = q.z();
        start_pose.orientation.w = q.w();
        start_pose.position.y -= 0.6;
        waypoints.push_back(start_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        try{

            ws_trajectory = new moveit_dp_redundancy_resolution::WorkspaceTrajectory(
                    SQUARE_TRAJECTORY,
                    trajectory,
                    move_group.getCurrentState(),
                    PLANNING_GROUP);

        }catch(ros::Exception & e)
        {
            ROS_ERROR_NAMED("main", "%s", e.what());
            ros::shutdown();
            exit(1);
        }

        // Be aware that the computeCartesianPath function may assign non-uniform timestamps. By doing so,
        // it guarantees that two neighbor waypoints corresponding to "far" joint-space configurations
        // can be tracked by the robot without violating its velocity limits. If you want to avoid such a
        // behavior, you can set evenly spaced timestamps.

        ws_trajectory->setEvenlySpacedTimestamps(60.0);

        // Export the computed workspace trajectory so that it can be used later with the case LOADED_TRAJECTORY

        std::string package_path = ros::package::getPath("panda_controller");
        std::string trajectory_file = "square_trajectory.traj";
        std::string file_path = package_path + "/trajectories/" + trajectory_file;

        ws_trajectory->exportToBinary(file_path);

        ROS_INFO_NAMED("main", "Visualizing Cartesian path (%.2f%% achieved) as trajectory line", fraction * 100.0);

        visual_tools.deleteAllMarkers();
        visual_tools.publishPath(ws_trajectory->getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
        visual_tools.trigger();

        break;
    }
    case 3:
    {
        std::string package_path = ros::package::getPath("panda_controller");
        std::string trajectory_file = "square_trajectory.traj";
        std::string file_path = package_path + "/trajectories/" + trajectory_file;

        try{

            ws_trajectory = new moveit_dp_redundancy_resolution::WorkspaceTrajectory(
                    "square_trajectory_loaded",
                    file_path);

        }catch(ros::Exception & e)
        {
            ROS_ERROR_NAMED("main", "%s", e.what());
            ros::shutdown();
            exit(1);
        }

        ROS_INFO_NAMED("main", "Visualizing trajectory loaded from file");

        visual_tools.deleteAllMarkers();
        visual_tools.publishPath(ws_trajectory->getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
//        for (std::size_t i = 0; i < ws_trajectory->getLength(); ++i)
//            visual_tools.publishAxis(ws_trajectory->getWaypoints()[i], 0.05, 0.002);
        visual_tools.trigger();

        break;
    }
    default:
        ROS_ERROR("Could not recognize trajectory. Please check the trajectory.yaml configuration file in the /config folder of the panda_controller package.");
        ros::shutdown();
        exit(1);
    }

    ROS_INFO("Trajectory duration is %f seconds", ws_trajectory->getDuration());

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start planning");

    // We are now ready to solve the workspace trajectory with dynamic programming

    ros::ServiceClient dp_redundancy_resolution_service
        = node_handle.serviceClient<moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectory>(move_group::DP_REDUNDANCY_RESOLUTION_SERVICE_NAME);

    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryRequest req;
    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse res;
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;

    ws_trajectory->getWorkspaceTrajectoryMsg(ws_trajectory_msg);

    req.ws_trajectory = ws_trajectory_msg;
    req.planning_group_name = PLANNING_GROUP;
    req.redundancy_parameter_samples = 720;

    dp_redundancy_resolution_service.call(req, res);

    ROS_INFO("Solution has %lu points", res.solution.joint_trajectory.points.size());

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(res.solution, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to terminate the controller");

    if(ws_trajectory != NULL)
        delete ws_trajectory;

    ros::shutdown();
    exit(0);
}


