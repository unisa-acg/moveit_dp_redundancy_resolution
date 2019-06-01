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
 * Title:   objective_function.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Nov 21, 2018
 *
 * See objective_function.h for a description of the class.
 *
 * -------------------------------------------------------------------
 */

#include <moveit_dp_redundancy_resolution/objective_function.h>
#include <moveit_dp_redundancy_resolution/exceptions.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

const std::map<std::string, int> moveit_dp_redundancy_resolution::ObjectiveFunction::PERFORMANCE_INDEX_TYPE_MAP =
        {{"square_norm_velocities",     0},
        {"frame_obstacle_distance",     1}};


moveit_dp_redundancy_resolution::ObjectiveFunction::ObjectiveFunction(const std::string & description_file)
{
    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(description_file, tree);


    try{

        std::string criterion_str = tree.get<std::string>("objective_function.<xmlattr>.criterion");

        if(criterion_str == "minimize")
            criterion_ = OptimizationCriterion::minimize;
        else
            if(criterion_str == "maximize")
                criterion_ = OptimizationCriterion::maximize;
            else
                moveit_dp_redundancy_resolution::Exception("Unrecognized optimization criterion: it must be either 'minimize' or 'maximize'");

    }catch(boost::property_tree::ptree_bad_path & e)
    {
        ROS_ERROR("%s", e.what());
        throw moveit_dp_redundancy_resolution::Exception("The objective_function tag must exist and must have a 'criterion' attribute");
    }

    double total_weight = 0;

    BOOST_FOREACH(boost::property_tree::ptree::value_type const& v, tree.get_child("objective_function"))
    {
        if(v.first == "performance_index")
        {
            try{
                std::string pi_type = v.second.get<std::string>("<xmlattr>.type");
                PerformanceIndex * pi;

                switch(PERFORMANCE_INDEX_TYPE_MAP.at(pi_type))
                {
                case 0:
                    {
                        pi = new SquareNormVelocities(v.second);
                        p_indices_.push_back(pi);
                        break;
                    }
                case 1:
                    {
                        pi = new FrameObstacleDistance(v.second);
                        p_indices_.push_back(pi);
                        break;
                    }
                default:
                    throw moveit_dp_redundancy_resolution::Exception("Unrecognized performance index type");

                }

            }catch(boost::property_tree::ptree_bad_path & e)
            {
                ROS_ERROR("%s", e.what());
                throw moveit_dp_redundancy_resolution::Exception("Each performance_index tag must have a 'type' attribute");
            }

            total_weight += p_indices_[p_indices_.size()-1]->getWeight();
        }
    }

    if(total_weight != 1.0)
        throw moveit_dp_redundancy_resolution::Exception("Performance indices weights do not sum to one");

}

moveit_dp_redundancy_resolution::ObjectiveFunction::~ObjectiveFunction()
{
    std::vector<PerformanceIndex *>::iterator iter = p_indices_.begin();

    for(; iter != p_indices_.end(); iter++)
        delete *iter;
}

bool moveit_dp_redundancy_resolution::ObjectiveFunction::isCriterionMinimize() const
{
    if(criterion_ == OptimizationCriterion::minimize)
        return true;
    else
        return false;
}

bool moveit_dp_redundancy_resolution::ObjectiveFunction::isCriterionMaximize() const
{
    return !isCriterionMinimize();
}

double moveit_dp_redundancy_resolution::ObjectiveFunction::computeLocalCost(
        const robot_state::RobotState * rs,
        const moveit::core::JointModelGroup * jmg,
        double time_step) const
{
    std::vector<PerformanceIndex *>::const_iterator iter = p_indices_.begin();

    double cost = 0;

    for(; iter != p_indices_.end(); iter++)
    {
        cost += (*iter)->getWeight() * (*iter)->computeLocalCost(rs, jmg, time_step);
    }

    return cost;
}

moveit_dp_redundancy_resolution::PerformanceIndex::PerformanceIndex(const boost::property_tree::ptree & tree)
{
    try{

        weight_ = tree.get<double>("<xmlattr>.weight");

    }catch(boost::property_tree::ptree_bad_path & e)
    {
        ROS_ERROR("%s", e.what());
        throw moveit_dp_redundancy_resolution::Exception("Each performance_index tag must have a 'weight' attribute");
    }
}

double moveit_dp_redundancy_resolution::PerformanceIndex::getWeight() const
{
    return weight_;
}

moveit_dp_redundancy_resolution::SquareNormVelocities::SquareNormVelocities(const boost::property_tree::ptree & tree):
        PerformanceIndex(tree)
{
}

double moveit_dp_redundancy_resolution::SquareNormVelocities::computeLocalCost(
        const robot_state::RobotState * rs,
        const moveit::core::JointModelGroup * jmg,
        double time_step) const
{
    Eigen::VectorXd qd_curr;

    rs->copyJointGroupVelocities(jmg, qd_curr);

    return time_step * qd_curr.dot(qd_curr);
}

moveit_dp_redundancy_resolution::FrameObstacleDistance::FrameObstacleDistance(const boost::property_tree::ptree & tree):
        PerformanceIndex(tree)
{
    try{

        frame_name_ = tree.get<std::string>("frame");
        obstacle_pos_ <<  tree.get<double>("obstacle.x"), tree.get<double>("obstacle.y"), tree.get<double>("obstacle.z");

    }catch(boost::property_tree::ptree_bad_path & e)
    {
        ROS_ERROR("%s", e.what());
        throw moveit_dp_redundancy_resolution::Exception("The frame_obstacle_distance performance index requires the name of a frame fixed to the robot and the coordinates of the obstacle (x,y,z) in the world reference frame");
    }
}

double moveit_dp_redundancy_resolution::FrameObstacleDistance::computeLocalCost(
        const robot_state::RobotState * rs,
        const moveit::core::JointModelGroup * jmg,
        double time_step) const
{
    std::vector<std::string> names = jmg->getActiveJointModelNames();

    const Eigen::Affine3d & transform = rs->getFrameTransform(frame_name_);

    Eigen::Vector3d joint_pos = transform.translation();

    Eigen::Vector3d distance = joint_pos - obstacle_pos_;

    return time_step * distance.dot(distance);

}

