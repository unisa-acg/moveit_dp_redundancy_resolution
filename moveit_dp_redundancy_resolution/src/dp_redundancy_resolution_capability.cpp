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
 * Title:   dp_redundancy_resolution_capability.cpp 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 20, 2018
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <moveit_dp_redundancy_resolution/dp_redundancy_resolution_capability.h>
#include <moveit_dp_redundancy_resolution/dynamic_programming_solver.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/package.h>
#include <rosbag/bag.h>

const std::string move_group::MoveGroupDPRedundancyResolutionService::PLOT_PATH_TOPIC = "plot_planned_path";
const std::map<std::string, move_group::MoveGroupDPRedundancyResolutionService::RobotType> move_group::MoveGroupDPRedundancyResolutionService::ROBOT_TYPE_MAP = boost::assign::map_list_of("planar", planar)("spherical", spherical)("regional", regional)("spatial", spatial);

move_group::MoveGroupDPRedundancyResolutionService::MoveGroupDPRedundancyResolutionService():
    MoveGroupCapability("DPRedundancyResolutionService"),
    plot_computed_paths_(true),
    res_(NULL),
    req_(NULL),
    objective_function_(NULL)
{
}

move_group::MoveGroupDPRedundancyResolutionService::~MoveGroupDPRedundancyResolutionService()
{
    if(objective_function_!=NULL)
        delete objective_function_;
}

void move_group::MoveGroupDPRedundancyResolutionService::initialize()
{
    display_path_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);

    plot_path_ = node_handle_.advertise<trajectory_msgs::JointTrajectoryPoint>(PLOT_PATH_TOPIC, 10000, true);

    cartesian_path_service_ = root_node_handle_.advertiseService(move_group::DP_REDUNDANCY_RESOLUTION_SERVICE_NAME, &MoveGroupDPRedundancyResolutionService::computeService_, this);

    // Composing the file name for the grids and maps to be exported to binary file later

    std::string output_path;

    node_handle_.getParam("/move_group/dp_redundancy_solver_config/output_path", output_path);

    file_path_grids_base_ = output_path + "/grids/";
    file_path_maps_base_ = output_path + "/colormaps/";
    file_path_bags_base_ = output_path + "/bagfiles/";

    if(!boost::filesystem::is_directory(file_path_grids_base_))
        boost::filesystem::create_directory(file_path_grids_base_);

    if(!boost::filesystem::is_directory(file_path_maps_base_))
        boost::filesystem::create_directory(file_path_maps_base_);

    if(!boost::filesystem::is_directory(file_path_bags_base_))
        boost::filesystem::create_directory(file_path_bags_base_);

    // Initializing the ObjectiveFunction object needed by the DP solver later on

    std::string obj_function_path;

    node_handle_.getParam("/move_group/dp_redundancy_solver_config/objective_function", obj_function_path);

    objective_function_ = new moveit_dp_redundancy_resolution::ObjectiveFunction(obj_function_path);
}

bool move_group::MoveGroupDPRedundancyResolutionService::computeService_(moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryRequest & req,
                                                                         moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & res)
{
    ROS_INFO("Received request to solve redundancy with dynamic programming");

    req_ = &req;
    res_ = &res;

    // Checking a few things before starting redundancy resolution

    if(req_->planning_group_name == "")
    {
        ROS_ERROR("Planning group name is empty. Returning with no planned path.");
        return false;
    }

    if(!checkRedundancyParameters_())
        return false;

    if(req_->redundancy_parameter_samples == 0)
    {
        ROS_ERROR("Invalid number of samples for the redundancy parameter(s)");
        return false;
    }

    // Getting the robot type from the parameter server

    std::string robot_type;

    node_handle_.getParam("/move_group/dp_redundancy_solver_config/robot_type", robot_type);

    // Getting the robot kinematic model from the context

    const robot_model::RobotModelConstPtr robot_model = context_->planning_scene_monitor_->getRobotModel();

    // Sampling the redundancy parameter with the required number of samples

    const robot_model::VariableBounds rp_bounds = robot_model->getVariableBounds(req_->redundancy_parameters[0]);

    std::vector<double> rp_vector(req_->redundancy_parameter_samples);

    moveit_dp_redundancy_resolution::generateRange(rp_vector, rp_bounds.min_position_, rp_bounds.max_position_, req_->redundancy_parameter_samples);

    // Creating the output optimal robot trajectory

    robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req_->planning_group_name);

    // Computing globally-optimal solution with dynamic programming

    bool success = false;

    switch(ROBOT_TYPE_MAP.at(robot_type))
    {
    case RobotType::planar:
        success = computeOptimalSolutionPlanar_(rp_vector, rt);
        break;
    case RobotType::regional:
        ROS_ERROR("Optimal redundancy_resolution for regional robots is not implemented");
        return false;
    case RobotType::spatial:
        success = computeOptimalSolutionSpatial_(rp_vector, rt);
        break;
    default:
        ROS_ERROR("Unknown robot type. The robot_type parameter might be not defined on the parameter server or might have a wrong value.");
        return false;
    }

    if(success)
    {
        // Filling up the envelope

        rt.getRobotTrajectoryMsg(res.solution);

        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

        // Publishing the message to visualize the joint space trajectory in the 3D visualizer (e.g. RViz)

        moveit_msgs::DisplayTrajectory disp = publishTrajectoryForDisplay_(rt, res);

        // Publishing the messages to plot the joint space trajectory in a XY visualizer (e.g. rqt_plot)

        publishTrajectoryForPlot_(res);

        // Generate a bag file for replaying this solution later if needed

        publishTrajectoryToBag_(res, disp, file_path_bags_base_);

        return true;
    }
    else
        return false;
}

bool move_group::MoveGroupDPRedundancyResolutionService::computeOptimalSolutionPlanar_(
        const std::vector<double> & rp_vector,
        robot_trajectory::RobotTrajectory & robot_trajectory_out)
{
    ROS_INFO_NAMED("MoveGroupDPRedundancyResolutionService", "Solving redundancy with two grids");

    // Getting the robot kinematic model from the context

    const robot_model::RobotModelConstPtr robot_model = context_->planning_scene_monitor_->getRobotModel();

    std::vector<std::string> file_path_grids;

    file_path_grids.push_back(file_path_grids_base_ + req_->ws_trajectory.name + "01.grid");
    file_path_grids.push_back(file_path_grids_base_ + req_->ws_trajectory.name + "02.grid");


    // Creating the elbow right grid

    moveit_dp_redundancy_resolution::StateSpaceGrid2D ss_grid_er(req_->redundancy_parameters[0],
                                                                 robot_model->getJointModelGroup(req_->planning_group_name),
                                                                 robot_model->getJointModelGroup(req_->non_redundant_group_name),
                                                                 req_->ws_trajectory.waypoints,
                                                                 rp_vector,
                                                                 file_path_grids[0]);

    moveit::core::VariableBounds bounds;

    bounds.min_position_ = 0;
    bounds.max_position_ = M_PI;

    ss_grid_er.setJointLimits("joint3", bounds);

    try{

        ss_grid_er.computeGrid();

        ss_grid_er.exportToBinary();

        ss_grid_er.exportToMaps(file_path_maps_base_ + req_->ws_trajectory.name + "01_");

    }catch(moveit_dp_redundancy_resolution::Exception & e)
    {
        ROS_ERROR("%s", e.what());
        res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    // Creating the elbow left grid

    moveit_dp_redundancy_resolution::StateSpaceGrid2D ss_grid_el(req_->redundancy_parameters[0],
                                                                 robot_model->getJointModelGroup(req_->planning_group_name),
                                                                 robot_model->getJointModelGroup(req_->non_redundant_group_name),
                                                                 req_->ws_trajectory.waypoints,
                                                                 rp_vector,
                                                                 file_path_grids[1]);

    bounds.min_position_ = -M_PI;
    bounds.max_position_ = 0;

    ss_grid_el.setJointLimits("joint3", bounds);

    try{

        ss_grid_el.computeGrid();

        ss_grid_el.exportToBinary();

        ss_grid_el.exportToMaps(file_path_maps_base_ + req_->ws_trajectory.name + "02_");

    }catch(moveit_dp_redundancy_resolution::Exception & e)
    {
        ROS_ERROR("%s", e.what());
        res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    // Solving with dynamic programming

    moveit_dp_redundancy_resolution::DynamicProgrammingSolver dp_solver(
            robot_model,
            robot_model->getJointModelGroup(req_->planning_group_name),
            *objective_function_);

    try{

        dp_solver.addGrid(&ss_grid_er);
        dp_solver.addGrid(&ss_grid_el);
        dp_solver.setTimeVector(req_->ws_trajectory.timestamps);

        if(!dp_solver.solve(robot_trajectory_out))
        {
            res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

    }catch(moveit_dp_redundancy_resolution::Exception & e)
    {
        ROS_ERROR("%s", e.what());
        res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    return true;
}

bool move_group::MoveGroupDPRedundancyResolutionService::computeOptimalSolutionSpatial_(
        const std::vector<double> & rp_vector,
        robot_trajectory::RobotTrajectory & robot_trajectory_out)
{
    ROS_INFO_NAMED("MoveGroupDPRedundancyResolutionService", "Solving redundancy with multigrid");

    // Getting the robot kinematic model from the context

    const robot_model::RobotModelConstPtr robot_model = context_->planning_scene_monitor_->getRobotModel();

    moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D multigrid(   req_->redundancy_parameters[0],
                                                                        robot_model->getJointModelGroup(req_->planning_group_name),
                                                                        req_->ws_trajectory.waypoints,
                                                                        rp_vector,
                                                                        RobotType::spatial,
                                                                        file_path_grids_base_ + req_->ws_trajectory.name);

    ROS_INFO_NAMED("MoveGroupDPRedundancyResolutionService", "Created multigrid of %d grids of size (%lu,%lu)", RobotType::spatial, req_->ws_trajectory.waypoints.size(), rp_vector.size());

    multigrid.computeGrids();

    try{

        multigrid.exportToBinary();

        multigrid.exportToMaps(file_path_maps_base_ + req_->ws_trajectory.name);

    }catch(moveit_dp_redundancy_resolution::Exception & e)
    {
        ROS_ERROR("%s", e.what());
        res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    // Solving with dynamic programming

    moveit_dp_redundancy_resolution::DynamicProgrammingSolver dp_solver(
            robot_model,
            robot_model->getJointModelGroup(req_->planning_group_name),
            *objective_function_);

    try{

        for(unsigned int i=0; i < RobotType::spatial; i++)
        {
            dp_solver.addGrid(multigrid[i]);
        }

        dp_solver.setTimeVector(req_->ws_trajectory.timestamps);

        if(!dp_solver.solve(robot_trajectory_out))
        {
            res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

    }catch(moveit_dp_redundancy_resolution::Exception & e)
    {
        ROS_ERROR("%s", e.what());
        res_->error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    return true;
}

bool move_group::MoveGroupDPRedundancyResolutionService::checkRedundancyParameters_()
{
    if(req_->redundancy_parameters.size() == 0)
    {
        // Let's check whether the redundancy parameters are defined in the planning group already

        const moveit::core::JointModelGroup * jmg = context_->planning_scene_monitor_->getRobotModel()->getJointModelGroup(req_->planning_group_name);

        std::vector<unsigned int> rj_indices;

        jmg->getGroupKinematics().first.solver_instance_->getRedundantJoints(rj_indices);

        for(int i=0; i<rj_indices.size(); i++)
        {
            req_->redundancy_parameters.push_back(jmg->getVariableNames()[rj_indices[i]]);
        }

        if(req_->redundancy_parameters.size() == 0)
        {
            ROS_ERROR("No redundancy parameter specified in the planning group. Returning with no planned path.");
            return false;
        }
    }

    if(req_->redundancy_parameters.size() > 2)
    {
        ROS_ERROR("Cannot plan with more than 2 redundancy parameters. Returning with no planned path.");
        return false;
    }

    if(req_->redundancy_parameters.size() > 1)
    {
        ROS_ERROR("Planning with 3-dimensional grids is not implemented yet. Returning with no planned path.");
        return false;
    }

    return true;
}

moveit_msgs::DisplayTrajectory move_group::MoveGroupDPRedundancyResolutionService::publishTrajectoryForDisplay_(
        const robot_trajectory::RobotTrajectory & trajectory,
        const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & res)
{
    ROS_INFO("Publishing trajectory for 3D visualization...");

    moveit_msgs::DisplayTrajectory disp;

    disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
    disp.trajectory.resize(1, res.solution);
    robot_state::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), disp.trajectory_start);
    display_path_.publish(disp);

    ROS_INFO("... Done!");

    return disp;
}

/*
 * Be aware that live listeners of the messages generated by this function may lose
 * some of the messages because the transmission rate is too high.
 *
 * If a live stream is needed soon after the solution is computed by the solver,
 * this function shall be modified to lower its rate.
 *
 * If a live stream is not needed, replay the bag file generated by the
 * publishTrajectoryToBag_ function.
 */

void move_group::MoveGroupDPRedundancyResolutionService::publishTrajectoryForPlot_(
        const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & response)
{
    if(plot_computed_paths_ && response.solution.joint_trajectory.points.size() > 0)
    {
        ROS_INFO("Publishing trajectory for 2D plots...");

        for(int i=0; i < response.solution.joint_trajectory.points.size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint jtp;

            jtp = response.solution.joint_trajectory.points[i];

            plot_path_.publish(jtp);
        }

        ROS_INFO("... Done!");
    }
}

void move_group::MoveGroupDPRedundancyResolutionService::publishTrajectoryToBag_(
        const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & response,
        const moveit_msgs::DisplayTrajectory & disp_traj_msg,
        const std::string & file_path_base)
{
    ROS_INFO("Writing trajectory messages to bag file...");

    std::string filename = nowStr_() + ".bag";

    rosbag::Bag bag(file_path_base + filename, rosbag::bagmode::Write);

    bag.write(node_handle_.resolveName(planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC), ros::Time::now(), disp_traj_msg);

    for(int i=0; i < response.solution.joint_trajectory.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint jtp;

        jtp = response.solution.joint_trajectory.points[i];

        bag.write(node_handle_.resolveName(PLOT_PATH_TOPIC), ros::Time::now(), jtp);
    }

    bag.close();

    ROS_INFO("Generated bag file %s", filename.c_str());
}

std::string move_group::MoveGroupDPRedundancyResolutionService::nowStr_()
{
    std::stringstream msg;
    const boost::posix_time::ptime now= boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupDPRedundancyResolutionService, move_group::MoveGroupCapability)
