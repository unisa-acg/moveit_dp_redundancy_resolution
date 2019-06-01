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
 * Title:   state_space_grid.h 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 23, 2018
 *
 * StateSpaceGrid2D is the class representing the grid containing, for
 * each waypoint, the inverse kinematics solutions. For each waypoint,
 * the number of inverse kinematics solutions depends on the number of
 * discrete values of the redundancy parameters (one of the joints in
 * this case, according to the joint selection paradigm), which is
 * a parameter provided at the construction of the object. Each
 * inverse kinematics solution is contained in a StateSpaceNode
 * object, which, together with that, carries additional data to ease
 * the task of solvers using the grid. The StateSpaceGrid2D class also
 * has functions for exporting the grid and additional representations
 * (e.g. colormaps) to files. As well, the class provides an import
 * function for grids.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_STATE_SPACE_GRID_H_
#define INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_STATE_SPACE_GRID_H_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_dp_redundancy_resolution/exceptions.h>

namespace moveit_dp_redundancy_resolution
{

class StateSpaceMultiGrid2D;
class StateSpaceGrid2D;

class StateSpaceNode
{
    friend class StateSpaceGrid2D;
    friend class StateSpaceMultiGrid2D;

public:

    StateSpaceNode(): cumulative_cost_(0), predecessor_(NULL), enabled_(false) {}

    void setPredecessor(StateSpaceNode * p)
    {
        predecessor_ = p;
    }

    void setCost(double cost)
    {
        cumulative_cost_ = cost;
    }

    bool enable()
    {
        if(joint_positions_.size() != 0)
            enabled_ = true;

        return enabled_;
    }

    const Eigen::VectorXd & getJointPositions() const
    {
        return joint_positions_;
    }

    StateSpaceNode * getPredecessor() const
    {
        if(predecessor_ == NULL)
            throw moveit_dp_redundancy_resolution::Exception("Node predecessor is not valid");
        else
            return predecessor_;
    }

    const double getCost() const
    {
        return cumulative_cost_;
    }

private:
    Eigen::VectorXd joint_positions_;
    StateSpaceNode * predecessor_;
    double cumulative_cost_;
    bool enabled_;

};

class StateSpaceGrid2D
{
    friend class StateSpaceMultiGrid2D;

public:

    StateSpaceGrid2D(   const std::string & redundant_joint,
                        const moveit::core::JointModelGroup *redundant_jmg,
                        const moveit::core::JointModelGroup *non_redundant_jmg,
                        const std::vector<geometry_msgs::Pose> & waypoints,
                        const std::vector<double> & rp_vector,
                        const std::string & file_path = "");

    ~StateSpaceGrid2D();

    void initializeCost(double cost);
    bool enableNode(unsigned int waypoint_idx, unsigned int parameter_idx);
    void setJointLimits(std::string joint_name, const moveit::core::VariableBounds & bounds);

    void computeGrid();

    void exportToBinary() const;
    void exportToMaps(const std::string & path) const;

    unsigned int getWaypointsCount() const;
    unsigned int getParameterCount() const;
    StateSpaceNode * getNodePtr(unsigned int waypoint_idx, unsigned int parameter_idx) const;
    StateSpaceNode * getMinimalCostNodePtr(unsigned int waypoint_idx) const;
    StateSpaceNode * getMaximalCostNodePtr(unsigned int waypoint_idx) const;

    bool isNodeValid(unsigned int waypoint_idx, unsigned int parameter_idx) const;
    bool isNodeEnabled(unsigned int waypoint_idx, unsigned int parameter_idx) const;

private:

    void getCostVectorAtWaypoint_(std::vector<double> & vector_out, unsigned int waypoint_idx) const;

    bool checkConstraints_(moveit::core::RobotState * rs,
                            const moveit::core::JointModelGroup * jmg,
                            const double *joint_group_variable_values);

    void printJointPositions_(const robot_state::RobotState * rs);
    void printJointPositions_(const std::vector<double> positions);

    void loadGridFromBinary_();


    moveit::core::GroupStateValidityCallbackFn constraint_checker_callback_;

    StateSpaceNode ** state_space_grid_;

    std::string redundant_joint_;
    const moveit::core::JointModelGroup * redundant_jmg_;
    const moveit::core::JointModelGroup * non_redundant_jmg_;
    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<double> rp_vector_;
    const std::string file_path_;

    moveit::core::JointBoundsVector constraints_;

};

class StateSpaceMultiGrid2D
{

public:

    StateSpaceMultiGrid2D(  const std::string & redundant_joint,
                            const moveit::core::JointModelGroup *redundant_jmg,
                            const std::vector<geometry_msgs::Pose> & waypoints,
                            const std::vector<double> & rp_vector,
                            unsigned int number_of_grids,
                            const std::string & file_path = "");

    ~StateSpaceMultiGrid2D();

    StateSpaceGrid2D * operator[](unsigned int i) const;

    void computeGrids();

    void exportToBinary() const;
    void exportToMaps(const std::string & path_base) const;

private:

    std::vector<StateSpaceGrid2D *> multigrid_;

    std::string redundant_joint_;
    const moveit::core::JointModelGroup * redundant_jmg_;
    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<double> rp_vector_;
    const std::string file_path_;

};

}



#endif /* INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_STATE_SPACE_GRID_H_ */
