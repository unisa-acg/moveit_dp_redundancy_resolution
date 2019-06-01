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
 * Title:   objective_function.h
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Nov 21, 2018
 *
 * This file contains a set of classes that are kept together as they
 * they are tightly related from the semantic point of view and must
 * be used together. The ObjectiveFunction class implements a weighted
 * sum of performance indices. Each performance index is implemented
 * by inheriting from the abstract class PerformanceIndex, which
 * requires the method computeLocalCost to be implemented. By default,
 * this library also provides some implementations, but more are
 * possible outside of this class. The ObjectiveFunction class is
 * directly used in the DynamicProgrammingSolver class, as it
 * transparently manages the computation of the objective function,
 * without requiring the DynamicProgrammingSolver to know how to do
 * it. The parameters of the single performance indices (if any), as
 * well as their weight, are loaded from an XML description file. The
 * ObjectiveFunction class is loaded when the
 * MoveGroupDPRedundancyResolutionService is loaded. Thereby it does
 * not require the objective function to be included in the redundancy
 * resolution request.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_OBJECTIVE_FUNCTION_H_
#define INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_OBJECTIVE_FUNCTION_H_

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/property_tree/ptree.hpp>

namespace moveit_dp_redundancy_resolution
{

/***************************************************
 *
 * Abstract class PerformanceIndex
 *
 ***************************************************/

class PerformanceIndex
{
public:
    PerformanceIndex(const boost::property_tree::ptree & tree);
    virtual ~PerformanceIndex(){};

    virtual double computeLocalCost(
            const robot_state::RobotState * rs,
            const moveit::core::JointModelGroup * jmg,
            double time_step) const = 0;

    double getWeight() const;

private:
    double weight_;
};

/***************************************************
 *
 * ObjectiveFunction class (works as a factory)
 *
 ***************************************************/

class ObjectiveFunction
{
    enum OptimizationCriterion
    {
        minimize,
        maximize
    };

    static const std::map<std::string, int> PERFORMANCE_INDEX_TYPE_MAP;

public:

    ObjectiveFunction(const std::string & description_file);
    ~ObjectiveFunction();

    double computeLocalCost(const robot_state::RobotState * rs,
                            const moveit::core::JointModelGroup * jmg,
                            double time_step) const;

    bool isCriterionMinimize() const;
    bool isCriterionMaximize() const;

private:

    std::vector<PerformanceIndex *> p_indices_;
    OptimizationCriterion criterion_;
};

/***************************************************
 *
 * Performance indices implementations
 *
 ***************************************************/

class SquareNormVelocities : public PerformanceIndex
{
public:
    SquareNormVelocities(const boost::property_tree::ptree & tree);

    double computeLocalCost(const robot_state::RobotState * rs,
                            const moveit::core::JointModelGroup * jmg,
                            double time_step) const;
};

class FrameObstacleDistance : public PerformanceIndex
{
public:
    FrameObstacleDistance(const boost::property_tree::ptree & tree);

    double computeLocalCost(const robot_state::RobotState * rs,
                            const moveit::core::JointModelGroup * jmg,
                            double time_step) const;

private:

    std::string frame_name_;
    Eigen::Vector3d obstacle_pos_;
};

}



#endif /* INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_OBJECTIVE_FUNCTION_H_ */
