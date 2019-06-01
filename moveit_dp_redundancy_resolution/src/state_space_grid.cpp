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
 * Title:   state_space_grid.cpp 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 23, 2018
 *
 * See state_space_grid.h for a description of the classes herein.
 *
 * -------------------------------------------------------------------
 */

#include <moveit_dp_redundancy_resolution/state_space_grid.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/********************************************************************
 *                                                                  *
 *                  StateSpaceGrid2D functions                      *
 *                                                                  *
 ********************************************************************/

moveit_dp_redundancy_resolution::StateSpaceGrid2D::StateSpaceGrid2D(
        const std::string & redundant_joint,
        const moveit::core::JointModelGroup *redundant_jmg,
        const moveit::core::JointModelGroup *non_redundant_jmg,
        const std::vector<geometry_msgs::Pose> & waypoints,
        const std::vector<double> & rp_vector,
        const std::string & file_path):
    redundant_joint_(redundant_joint),
    redundant_jmg_(redundant_jmg),
    non_redundant_jmg_(non_redundant_jmg),
    waypoints_(waypoints),
    rp_vector_(rp_vector),
    file_path_(file_path),
    constraints_(non_redundant_jmg_ == NULL ? 0 : non_redundant_jmg_->getVariableCount())
{
    state_space_grid_ = new StateSpaceNode* [waypoints_.size()];
    for(int i=0; i < waypoints_.size(); i++)
        state_space_grid_[i] = new StateSpaceNode[rp_vector_.size()];

    constraint_checker_callback_ = boost::bind(&moveit_dp_redundancy_resolution::StateSpaceGrid2D::checkConstraints_,
                                         this, _1, _2, _3);

    if(non_redundant_jmg_ != NULL)
    {
        for(int i=0; i < constraints_.size(); i++)
        {
            moveit::core::JointModel::Bounds * bounds_ptr = new moveit::core::JointModel::Bounds(*(non_redundant_jmg_->getActiveJointModelsBounds()[i]));

            constraints_[i] = bounds_ptr;
        }
    }
}

moveit_dp_redundancy_resolution::StateSpaceGrid2D::~StateSpaceGrid2D()
{
    for(int i=0; i < waypoints_.size(); i++)
    {
        delete[] state_space_grid_[i];
    }
    delete[] state_space_grid_;

    for(int i=0; i < constraints_.size(); i++)
    {
        delete constraints_[i];
    }
}


void moveit_dp_redundancy_resolution::StateSpaceGrid2D::initializeCost(double cost)
{
    for(int i=0; i < waypoints_.size(); i++)
        for(int j=0; j < rp_vector_.size(); j++)
            state_space_grid_[i][j].cumulative_cost_ = cost;
}

bool moveit_dp_redundancy_resolution::StateSpaceGrid2D::enableNode(
        unsigned int waypoint_idx,
        unsigned int parameter_idx)
{
    if(isNodeValid(waypoint_idx, parameter_idx))
        state_space_grid_[waypoint_idx][parameter_idx].enabled_ = true;

    return state_space_grid_[waypoint_idx][parameter_idx].enabled_;
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::setJointLimits(
        std::string joint_name,
        const moveit::core::VariableBounds & bounds)
{
    unsigned int index = non_redundant_jmg_->getVariableGroupIndex(joint_name);

    delete constraints_[index];

    std::vector<moveit::core::VariableBounds> * bounds_vector = new std::vector<moveit::core::VariableBounds>();

    bounds_vector->push_back(bounds);

    constraints_[index] = bounds_vector;
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::computeGrid()
{
    ROS_INFO("Computing state space grid...");

    if(file_path_ == "" || !boost::filesystem::exists(file_path_))
    {
        ROS_INFO("Computing grid...");

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

        robot_state::RobotState rs(kinematic_model);

        for(int j=0; j<rp_vector_.size(); j++)
        {
            rs.setJointPositions(redundant_joint_, &(rp_vector_[j]));

            ROS_INFO("Computing solutions for sample %d", j);

            for(int i=0; i<waypoints_.size(); i++)
            {
                bool solution_found = rs.setFromIK(non_redundant_jmg_, waypoints_[i], 5, 0.01, constraint_checker_callback_);

                if(solution_found)
                    rs.copyJointGroupPositions(redundant_jmg_->getName(), state_space_grid_[i][j].joint_positions_);

//                if(solution_found)
//                    ROS_INFO("(%d,%d) Found a solution", i, j);
//                else
//                    ROS_INFO("(%d,%d) Could not find a solution", i, j);
            }
        }

        ROS_INFO("... Done!");
    }
    else
    {
        ROS_INFO("Grid already exists: loading from file...");

        loadGridFromBinary_();

        ROS_INFO("Loaded a grid of size %d x %d", getWaypointsCount(), getParameterCount() );
    }

}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::exportToBinary() const
{
    if(file_path_ == "")
        throw moveit_dp_redundancy_resolution::Exception("File path is not set");

    if(boost::filesystem::exists(file_path_))
    {
        ROS_WARN("Skipping export to binary: grid already exists");
        return;
    }

    std::ofstream out_file(file_path_, std::ofstream::out | std::ofstream::binary);

    if(out_file.rdstate() != std::ios_base::goodbit)
        throw moveit_dp_redundancy_resolution::Exception("Could not create the output file");

    ROS_INFO("Exporting the grid to binary file %s...", file_path_.c_str());

    unsigned long wp_count = waypoints_.size();
    unsigned long rp_count = rp_vector_.size();

    out_file.write((char*)&wp_count, sizeof(unsigned long));
    out_file.write((char*)&rp_count, sizeof(unsigned long));

    for(int i=0; i < wp_count; i++)
        for(int j=0; j < rp_count; j++)
        {
            Eigen::VectorXd joint_positions = state_space_grid_[i][j].joint_positions_;

            if(joint_positions.size() == redundant_jmg_->getVariableCount())
            {
                out_file << 'd';

                for(int k=0; k<joint_positions.size(); k++)
                    out_file.write( (char*)&(joint_positions[k]), sizeof(double));
            }
            else
            {
                out_file << 'n';
            }
        }

    out_file.close();

    ROS_INFO("... Done!");
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::exportToMaps(
        const std::string & path) const
{
    ROS_INFO("Received request to export the grid to maps");

    const std::vector<int> & index_list = redundant_jmg_->getVariableIndexList();

    unsigned int rj_index = redundant_jmg_->getVariableGroupIndex(redundant_joint_);

    std::vector<int>::const_iterator index = index_list.begin();

    unsigned int rows = rp_vector_.size();
    unsigned int cols = waypoints_.size();

    // Creating a map for each joint in the non-redundant chain

    for(; index != index_list.end(); ++index)
    {
        if(*index != rj_index)
        {
            ROS_INFO("Exporting %s map", redundant_jmg_->getVariableNames()[*index].c_str());

            // Extracting the joint positions from the grid and building a matrix of double

            cv::Mat image_d(rows, cols, CV_64FC1, std::numeric_limits<double>::quiet_NaN());

            for(int i=0; i < cols; i++)
                for(int j=0; j < rows; j++)
                {
                    if(state_space_grid_[i][j].joint_positions_.size() == redundant_jmg_->getVariableCount())
                        image_d.at<double>(j,i) = state_space_grid_[i][j].joint_positions_[*index];
                }

            // Converting the matrix of double to grayscale, then applying the color map

            cv::Mat image(rows, cols, CV_8UC1, std::numeric_limits<double>::quiet_NaN());
            cv::Mat color_map;

            image_d.convertTo(image, CV_8UC1, 128/M_PI, 128);
            cv::applyColorMap(image, color_map, cv::COLORMAP_HSV);

            // Coloring NaNs with white

            for(int i=0; i<cols; i++)
                for(int j=0; j<rows; j++)
                    if(image_d.at<double>(j,i) != image_d.at<double>(j,i))
                    {
                        color_map.at<cv::Vec3b>(j,i)[0] = 255;
                        color_map.at<cv::Vec3b>(j,i)[1] = 255;
                        color_map.at<cv::Vec3b>(j,i)[2] = 255;
                    }

            // Exporting to file

            std::string file_path = path + redundant_jmg_->getVariableNames()[*index] + ".png";

            std::vector<int> params;

            params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            params.push_back(0);

            bool ok = cv::imwrite(file_path, color_map, params);

            if(!ok)
                throw moveit_dp_redundancy_resolution::Exception("Could not export the grid to file");

            // Uncomment to display images

//            cv::imshow("doubleMap", image_d);
//            cv::imshow("convertedMap", image);
//            cv::imshow("colorMap", color_map);
//            cv::waitKey(0);
        }
    }

//    cv::destroyWindow("doubleMap");
//    cv::destroyWindow("convertedMap");
//    cv::destroyWindow("colorMap");

}

unsigned int moveit_dp_redundancy_resolution::StateSpaceGrid2D::getWaypointsCount() const
{
    return waypoints_.size();
}

unsigned int moveit_dp_redundancy_resolution::StateSpaceGrid2D::getParameterCount() const
{
    return rp_vector_.size();
}

moveit_dp_redundancy_resolution::StateSpaceNode * moveit_dp_redundancy_resolution::StateSpaceGrid2D::getNodePtr(
        unsigned int waypoint_idx,
        unsigned int parameter_idx) const
{
    if(waypoint_idx >= waypoints_.size() || parameter_idx >= rp_vector_.size())
        throw moveit_dp_redundancy_resolution::Exception("Indices are out of bounds");

    if(!isNodeValid(waypoint_idx, parameter_idx))
        throw moveit_dp_redundancy_resolution::Exception("Cannot return an invalid node");

    return &state_space_grid_[waypoint_idx][parameter_idx];
}

moveit_dp_redundancy_resolution::StateSpaceNode * moveit_dp_redundancy_resolution::StateSpaceGrid2D::getMinimalCostNodePtr(
        unsigned int waypoint_idx) const
{
    std::vector<double> costs;

    getCostVectorAtWaypoint_(costs, waypoint_idx);

    unsigned int min_j = std::distance(costs.begin(), std::min_element(costs.begin(), costs.end()));

    if(isNodeValid(waypoint_idx, min_j))
        return getNodePtr(waypoint_idx, min_j);
    else
        return NULL;
}

moveit_dp_redundancy_resolution::StateSpaceNode * moveit_dp_redundancy_resolution::StateSpaceGrid2D::getMaximalCostNodePtr(
        unsigned int waypoint_idx) const
{
    std::vector<double> costs;

    getCostVectorAtWaypoint_(costs, waypoint_idx);

    unsigned int max_j = std::distance(costs.begin(), std::max_element(costs.begin(), costs.end()));

    if(isNodeValid(waypoint_idx, max_j))
        return getNodePtr(waypoint_idx, max_j);
    else
        return NULL;
}

bool moveit_dp_redundancy_resolution::StateSpaceGrid2D::isNodeValid(
        unsigned int waypoint_idx,
        unsigned int parameter_idx) const
{
    if(waypoint_idx >= waypoints_.size() || parameter_idx >= rp_vector_.size())
        throw moveit_dp_redundancy_resolution::Exception("Indices are out of bounds");

    if(state_space_grid_[waypoint_idx][parameter_idx].joint_positions_.size() == redundant_jmg_->getVariableCount())
        return true;
    else
        return false;
}

bool moveit_dp_redundancy_resolution::StateSpaceGrid2D::isNodeEnabled(
        unsigned int waypoint_idx,
        unsigned int parameter_idx) const
{
    if(waypoint_idx >= waypoints_.size() || parameter_idx >= rp_vector_.size())
        throw moveit_dp_redundancy_resolution::Exception("Indices are out of bounds");

    return state_space_grid_[waypoint_idx][parameter_idx].enabled_;
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::getCostVectorAtWaypoint_(
        std::vector<double> & vector_out,
        unsigned int waypoint_idx) const
{
    if(waypoint_idx >= waypoints_.size())
        throw moveit_dp_redundancy_resolution::Exception("Index exceeds grid size");

    for(int j=0; j<rp_vector_.size(); j++)
        vector_out.push_back(state_space_grid_[waypoint_idx][j].getCost());
}

bool moveit_dp_redundancy_resolution::StateSpaceGrid2D::checkConstraints_(
        moveit::core::RobotState * rs,
        const moveit::core::JointModelGroup * jmg,
        const double * joint_group_variable_values)
{
    return jmg->satisfiesPositionBounds(joint_group_variable_values, constraints_);
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::printJointPositions_(
        const robot_state::RobotState * rs)
{
    const std::vector<std::string> joint_names = rs->getVariableNames();

    std::ostringstream stream;

    stream.precision(3);

    for(int i=0; i<joint_names.size(); i++)
    {
        const double * position = rs->getJointPositions(joint_names[i]);

        stream << "q" << (i+1) << ": " << *position << " ";
    }

    ROS_INFO("%s", stream.str().c_str());
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::printJointPositions_(
        const std::vector<double> positions)
{
    std::ostringstream stream;

    stream.precision(3);

    for(int i=0; i < positions.size(); i++)
    {
        stream << "q" << (i+1) << ": " << positions[i] << " ";
    }

    ROS_INFO("%s", stream.str().c_str());
}

void moveit_dp_redundancy_resolution::StateSpaceGrid2D::loadGridFromBinary_()
{
    if(file_path_ == "" || !boost::filesystem::exists(file_path_))
        throw moveit_dp_redundancy_resolution::Exception("File path is not set or the file does not exist");

    std::ifstream in_file(file_path_, std::ifstream::in | std::ifstream::binary);

    unsigned long wp_count, rp_count;

    in_file.read((char*)&wp_count, sizeof(unsigned long));
    in_file.read((char*)&rp_count, sizeof(unsigned long));

    if(wp_count != waypoints_.size() || rp_count != rp_vector_.size())
        throw moveit_dp_redundancy_resolution::Exception("A grid exists for this trajectory but its size is not the same as requested.");

    for(int i=0; i<waypoints_.size(); i++)
        for(int j=0; j<rp_vector_.size(); j++)
        {
            char nan_or_double;

            in_file >> nan_or_double;

            if(nan_or_double == 'd')
            {
                state_space_grid_[i][j].joint_positions_.resize(redundant_jmg_->getVariableCount());

                for(int k=0; k < redundant_jmg_->getVariableCount(); k++)
                {
                    double tmp;
                    in_file.read((char*)&(tmp), sizeof(double));
                    state_space_grid_[i][j].joint_positions_[k] = tmp;
                }
            }
        }

    in_file.close();
}

/********************************************************************
 *                                                                  *
 *               StateSpaceMultiGrid2D functions                    *
 *                                                                  *
 ********************************************************************/


moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::StateSpaceMultiGrid2D(
        const std::string & redundant_joint,
        const moveit::core::JointModelGroup *redundant_jmg,
        const std::vector<geometry_msgs::Pose> & waypoints,
        const std::vector<double> & rp_vector,
        unsigned int number_of_grids,
        const std::string & file_path):
    redundant_joint_(redundant_joint),
    redundant_jmg_(redundant_jmg),
    waypoints_(waypoints),
    rp_vector_(rp_vector),
    file_path_(file_path)
{

    for(int i=0; i < number_of_grids; i++)
    {
        std::string single_grid_file_path = (file_path_ == "" ? file_path_ : file_path_ + "_" + std::to_string(i+1) + ".grid");

        StateSpaceGrid2D * grid = new StateSpaceGrid2D(redundant_joint_, redundant_jmg_, NULL, waypoints_, rp_vector_, single_grid_file_path);

        multigrid_.push_back(grid);
    }
}

moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::~StateSpaceMultiGrid2D()
{
    for(int k=0; k<multigrid_.size(); k++)
        delete multigrid_[k];
}

moveit_dp_redundancy_resolution::StateSpaceGrid2D * moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::operator[](unsigned int i) const
{
    return multigrid_[i];
}

void moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::computeGrids()
{
    ROS_INFO("Computing multiple grids...");

    const std::pair< moveit::core::JointModelGroup::KinematicsSolver, moveit::core::JointModelGroup::KinematicsSolverMap > group_kinematics = redundant_jmg_->getGroupKinematics();

    moveit::core::JointModelGroup::KinematicsSolver kinematics_solver = group_kinematics.first;

    kinematics::KinematicsBasePtr solver = kinematics_solver.solver_instance_;

    unsigned int redundant_variable_index = redundant_jmg_->getVariableGroupIndex(redundant_joint_);

    kinematics::KinematicsQueryOptions options;
    options.lock_redundant_joints = true;

    for(int j=0; j<rp_vector_.size(); j++)
    {
        ROS_INFO("Computing solutions for sample %d", j);

        std::vector<double> seed_state(redundant_jmg_->getVariableCount());

        seed_state[redundant_variable_index] = rp_vector_[j];

        seed_state.push_back(0);

//        ROS_INFO("Seed state:");
//        for(int k=0; k<seed_state.size(); k++)
//            ROS_INFO("%f", seed_state[k]);

        for(int i=0; i<waypoints_.size(); i++)
        {
            std::vector<geometry_msgs::Pose> ee_pose;
            std::vector<std::vector<double>> solutions;
            kinematics::KinematicsResult result;

            ee_pose.push_back(waypoints_[i]);

//            ROS_INFO("%f", ee_pose[0].position.x);
//            ROS_INFO("%f", ee_pose[0].position.y);
//            ROS_INFO("%f", ee_pose[0].position.z);
//            ROS_INFO("%f", ee_pose[0].orientation.x);
//            ROS_INFO("%f", ee_pose[0].orientation.y);
//            ROS_INFO("%f", ee_pose[0].orientation.z);

            bool solution_found = solver->getPositionIK(ee_pose, seed_state, solutions, result, options);

//            ROS_INFO("Waypoint %d: found %lu IK solutions", i, solutions.size());
//            ROS_INFO("Result = %d", result.kinematic_error);

            if(solution_found)
            {
                for(int k=0; k<solutions.size(); k++)
                {
                    multigrid_[k]->state_space_grid_[i][j].joint_positions_.resize(solutions[k].size());

//                    ROS_INFO("Solution: ");

                    for(int l=0; l<solutions[k].size(); l++)
                    {
                        multigrid_[k]->state_space_grid_[i][j].joint_positions_(l) = solutions[k][l];
//                        ROS_INFO("%f", multigrid_[k]->state_space_grid_[i][j].joint_positions_(l));
                    }
                }
            }

        }
    }
}

void moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::exportToBinary() const
{
    for(int k=0; k<multigrid_.size(); k++)
    {
        multigrid_[k]->exportToBinary();
    }
}

void moveit_dp_redundancy_resolution::StateSpaceMultiGrid2D::exportToMaps(
        const std::string & path_base) const
{
    for(int k=0; k<multigrid_.size(); k++)
    {
        multigrid_[k]->exportToMaps(path_base + "_" + std::to_string(k) + "_");
    }
}

