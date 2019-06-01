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
 * Title:   dynamic_programming_solver.cpp 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 30, 2018
 *
 * See dynamic_programming_solver.h for a description of the class.
 *
 * -------------------------------------------------------------------
 */

#include <moveit_dp_redundancy_resolution/dynamic_programming_solver.h>
#include <moveit_dp_redundancy_resolution/exceptions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <angles/angles.h>

moveit_dp_redundancy_resolution::DynamicProgrammingSolver::DynamicProgrammingSolver(
        const robot_model::RobotModelConstPtr & model,
        const moveit::core::JointModelGroup * jmg,
        const ObjectiveFunction & objective_function):
            model_(model),
            jmg_(jmg),
            objective_function_(objective_function),
            robot_state_(NULL)
{

}

moveit_dp_redundancy_resolution::DynamicProgrammingSolver::~DynamicProgrammingSolver()
{
    if(robot_state_ != NULL)
        delete robot_state_;
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::addGrid(StateSpaceGrid2D * grid)
{
    bool first_grid = (grids_.size() == 0);
    bool no_timestamps = (timestamps_.size() == 0);
    bool size_ok = false;

    if(!first_grid)
    {
        if(grids_[0]->getParameterCount() == grid->getParameterCount() &&
           grids_[0]->getWaypointsCount() == grid->getWaypointsCount())
            size_ok = true;
    }
    else
    {
        if(!no_timestamps && grid->getWaypointsCount() == timestamps_.size())
            size_ok = true;
    }

    if((first_grid && no_timestamps) || size_ok)
        grids_.push_back(grid);
    else
        throw moveit_dp_redundancy_resolution::Exception("Grid size does not match with previously set data");

    ROS_INFO("Added a state space grid to the dynamic programming solver");
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::setTimeVector(const std::vector<double> & timestamps)
{
    bool no_grid = (grids_.size() == 0);
    bool size_ok = false;

    if(!no_grid && timestamps.size() == grids_[0]->getWaypointsCount())
        size_ok = true;

    if(no_grid || size_ok)
        timestamps_ = timestamps;
    else
        throw moveit_dp_redundancy_resolution::Exception("Time vector size does not match grids size");

    ROS_INFO("Time vector is set");
}

bool moveit_dp_redundancy_resolution::DynamicProgrammingSolver::solve(
        robot_trajectory::RobotTrajectory & robot_trajectory)
{
    ROS_INFO("Solving redundancy with dynamic programming...");

    if(grids_.size() == 0)
        throw moveit_dp_redundancy_resolution::Exception("No grid is loaded");

    if(timestamps_.size() == 0)
        throw moveit_dp_redundancy_resolution::Exception("Time vector is not available");

    robot_state_ =  new robot_state::RobotState(model_);

    int waypoints_count = grids_[0]->getWaypointsCount();
    int rp_count = grids_[0]->getParameterCount();
    int grid_count = grids_.size();
    int last_waypoint;

    objective_function_.isCriterionMinimize() ? initializeGridsForMinimization_() : initializeGridsForMaximization_();

    enableAllInitialStates_();

    // For each waypoint/time step

    for(int i=0; i < waypoints_count-1; i++)
    {
        ROS_INFO("Waypoint %d/%d", i, waypoints_count-1);

        // For all nodes at this step

        for(int j=0; j<rp_count; j++)
            for(int k=0; k<grid_count; k++)
            {
                if(grids_[k]->isNodeEnabled(i,j))
                {
                    //ROS_INFO("Entering node (%d,%d) on grid %d", i, j, k);

                    last_waypoint = i;

                    StateSpaceNode * curr = grids_[k]->getNodePtr(i,j);
                    StateSpaceNode * prev;

                    assert(curr->getCost() != std::numeric_limits<double>::infinity());

                    if(i > 0)
                        prev = curr->getPredecessor();
                    else
                        prev = curr;

                    computeCostsFromNode_(i, j, prev, curr);
                }
            }
    }

    ROS_INFO("Waypoint %d/%d", waypoints_count-1, waypoints_count-1);

    if(last_waypoint != waypoints_count - 2)
    {
        ROS_WARN("Could not find a solution: stopped at waypoint %d", last_waypoint);
        return false;
    }

    // Collecting the solution

    StateSpaceNode * node = findLastNode_();

    ROS_INFO("Found a solution with cost I = %f", node->getCost());

    createRobotTrajectory_(robot_trajectory, node);

    return true;
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::initializeGridsForMaximization_()
{
    for(int k=0; k < grids_.size(); k++)
        grids_[k]->initializeCost(0);
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::initializeGridsForMinimization_()
{
    for(int k=0; k < grids_.size(); k++)
    {
        grids_[k]->initializeCost(std::numeric_limits<double>::infinity());

        for(int j=0; j < grids_[0]->getParameterCount(); j++)
            if(grids_[k]->isNodeValid(0,j))
                grids_[k]->getNodePtr(0,j)->setCost(0);
    }
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::enableAllInitialStates_()
{
    for(int k=0; k<grids_.size(); k++)
        for(int j=0; j<grids_[0]->getParameterCount(); j++)
            grids_[k]->enableNode(0,j);
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::computeCostsFromNode_(
        unsigned int stage,
        unsigned int rp_index,
        StateSpaceNode * prev,
        StateSpaceNode * node)
{
    if(stage == grids_[0]->getWaypointsCount())
        throw moveit_dp_redundancy_resolution::Exception("Stage " + std::to_string(stage) + " is already the last stage: cannot compute costs");

    int i = stage + 1;
    int j = rp_index;
    int max_j = grids_[0]->getParameterCount();
    bool stop_comparing = false;
    bool go_up = true;
    bool switch_direction = false;

    while(!stop_comparing)
    {
        std::vector<bool> satisfies_bounds(grids_.size(), true);

        for(int k=0; k < grids_.size(); k++)
        {
            if(grids_[k]->isNodeValid(i,j))
            {
                //ROS_INFO("Comparing with node (%d,%d) on grid %d", i,j,k);

                StateSpaceNode * next = grids_[k]->getNodePtr(i,j);

                double delta_t_curr = timestamps_[i] - timestamps_[i-1];
                double delta_t_prev;

                if(i == 1)
                    delta_t_prev = delta_t_curr;
                else
                    delta_t_prev = timestamps_[i-1] - timestamps_[i-2];

                Eigen::VectorXd qd_prev(node->getJointPositions().size());
                computeAngleVectorsDifference_(qd_prev, node->getJointPositions(), prev->getJointPositions());
                qd_prev = qd_prev/delta_t_prev;

                Eigen::VectorXd qd_curr(node->getJointPositions().size());
                computeAngleVectorsDifference_(qd_curr, next->getJointPositions(), node->getJointPositions());
                qd_curr = qd_curr/delta_t_curr;

                // Using delta_t_prev guarantees that accelerations respect the forward Euler approximation
                // which is also used to collect the final solution in createRobotTrajectory_(...).

                Eigen::VectorXd qdd_curr = (qd_curr-qd_prev)/delta_t_prev;

                robot_state_->setJointGroupPositions(jmg_->getName(), next->getJointPositions());
                robot_state_->setJointGroupVelocities(jmg_->getName(), qd_curr);
                robot_state_->setJointGroupAccelerations(jmg_->getName(), qdd_curr);
                robot_state_->updateLinkTransforms();

                if(!robot_state_->satisfiesBounds(jmg_))
                {
                    // for optimization, keep memory that the node at this grid did not satisfy bounds
                    // this optimization only works if grids are homogeneous

                    satisfies_bounds[k] = false;
                }
                else
                {
                    if(satisfiesAccelerationBounds_(robot_state_))
                    {
                        next->enable();

                        double local_cost = objective_function_.computeLocalCost(robot_state_, jmg_, delta_t_curr);
                        double tmp_cost = node->getCost() + local_cost;

                        bool found_better_cost = (objective_function_.isCriterionMinimize() ? (tmp_cost < next->getCost()) : (tmp_cost > next->getCost()));

                        if(found_better_cost)
                        {
                          next->setCost(tmp_cost);
                          next->setPredecessor(node);
                        }
                    }
                }

                if(k == grids_.size() - 1 && std::find(satisfies_bounds.begin(), satisfies_bounds.end(), true) == satisfies_bounds.end())
                {
                    switch_direction = true;
                }
            }
        }

        if(switch_direction && go_up)
        {
            // Going up, but a switch has been requested: reset redundancy parameter index

            go_up = false;
            switch_direction = false;

            j = rp_index;
        }

        if(switch_direction && !go_up)
        {
            // Going down, but a switch has been requested: stop comparing

            stop_comparing = true;
        }

        if(go_up && j < max_j - 1)
        {
            // Going up and there is still room: increase j

            j++;
        }

        if(go_up && j == max_j - 1)
        {
            // Going up but there is no more room: switch direction and reset redundancy parameter index

            go_up = false;
            j = rp_index;
        }

        if(!go_up && j > 0)
        {
            // Going down and there is still room: decrease j

            j--;
        }

        if(!go_up && j == 0)
        {
            // Going down but there is no more room: stop comparing

            stop_comparing = true;
        }

    }
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::computeAngleVectorsDifference_(
        Eigen::VectorXd & diff,
        const Eigen::VectorXd & minuend,
        const Eigen::VectorXd & subtrahend) const
{

    for(int i=0; i<minuend.size(); i++)
        diff[i] = angles::shortest_angular_distance(subtrahend[i], minuend[i]);

}

/**
 *  \brief Checks whether the robot state \e robot_state satisfies the accelerations bounds.
 *  This function is necessary because the current implementation of RobotState::satisfiesBounds(...)
 *  does not check the acceleration constraints.
 *
 *  \param robot_state. The RobotState with initialized accelerations.
 */
bool moveit_dp_redundancy_resolution::DynamicProgrammingSolver::satisfiesAccelerationBounds_(
        const robot_state::RobotState * robot_state) const
{
    if(robot_state == NULL)
        throw moveit_dp_redundancy_resolution::Exception("satisfiesAccelerationBounds_: RobotState is NULL");

    std::vector<std::string> names = jmg_->getActiveJointModelNames();

    for(int i=0; i<names.size(); i++)
    {
        const double * acceleration = robot_state->getJointAccelerations(names[i]);

        const moveit::core::VariableBounds & bounds = model_->getVariableBounds(names[i]);

        if(*acceleration > bounds.max_acceleration_ || *acceleration < bounds.min_acceleration_)
            return false;
    }

    return true;
}

moveit_dp_redundancy_resolution::StateSpaceNode * moveit_dp_redundancy_resolution::DynamicProgrammingSolver::findLastNode_()
{
    std::vector<double> costs;

    int last_idx = grids_[0]->getWaypointsCount() - 1;

    StateSpaceNode * best_node;

    for(int k=0; k < grids_.size(); k++)
    {
        best_node = (objective_function_.isCriterionMinimize() ?
                grids_[k]->getMinimalCostNodePtr(last_idx) :
                grids_[k]->getMaximalCostNodePtr(last_idx));

        if(best_node != NULL)
            costs.push_back(best_node->getCost());
        else
        {
            objective_function_.isCriterionMinimize() ?
                costs.push_back(std::numeric_limits<double>::infinity()) :
                costs.push_back(-std::numeric_limits<double>::infinity());
        }
    }

    unsigned int best_k = (objective_function_.isCriterionMinimize() ?
            std::distance(costs.begin(), std::min_element(costs.begin(), costs.end())) :
            std::distance(costs.begin(), std::max_element(costs.begin(), costs.end())));

    best_node = (objective_function_.isCriterionMinimize() ?
            grids_[best_k]->getMinimalCostNodePtr(last_idx) :
            grids_[best_k]->getMaximalCostNodePtr(last_idx));

    return best_node;
}

void moveit_dp_redundancy_resolution::DynamicProgrammingSolver::createRobotTrajectory_(
        robot_trajectory::RobotTrajectory & robot_trajectory,
        const StateSpaceNode * last_node)
{
    ROS_INFO("Collecting joint space trajectory...");

    unsigned int wp_count = grids_[0]->getWaypointsCount();

    std::vector<Eigen::VectorXd> joint_space_path(wp_count);
    std::vector<Eigen::VectorXd> velocities(wp_count);
    std::vector<Eigen::VectorXd> accelerations(wp_count);

    for(int i=wp_count-1; i >= 0; i--)
    {
        double delta_t;

        joint_space_path[i].resize(jmg_->getVariableCount());
        joint_space_path[i] = last_node->getJointPositions();

        velocities[i].resize(jmg_->getVariableCount());
        accelerations[i].resize(jmg_->getVariableCount());

        if(i > 0)
            last_node = last_node->getPredecessor();

        if(i < wp_count - 1)
        {
            delta_t = timestamps_[i+1] - timestamps_[i];

            computeAngleVectorsDifference_(velocities[i], joint_space_path[i+1], joint_space_path[i]);

            velocities[i] = velocities[i]/delta_t;
        }

        if(i < wp_count - 2)
        {
            accelerations[i] = (velocities[i+1] - velocities[i])/delta_t;

            robot_state_->setJointGroupPositions(jmg_, joint_space_path[i]);
            robot_state_->setJointGroupVelocities(jmg_, velocities[i]);
            robot_state_->setJointGroupAccelerations(jmg_, accelerations[i]);

            if(i == 0)
                robot_trajectory.addPrefixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(*robot_state_)), timestamps_[i]);
            else
                robot_trajectory.addPrefixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(*robot_state_)), timestamps_[i]-timestamps_[i-1]);
        }
    }

    velocities[wp_count - 1] = velocities[wp_count - 2];
    accelerations[wp_count - 2] = accelerations[wp_count - 3];
    accelerations[wp_count - 1] = accelerations[wp_count -2];

    robot_state_->setJointGroupPositions(jmg_, joint_space_path[wp_count - 2]);
    robot_state_->setJointGroupVelocities(jmg_, velocities[wp_count - 2]);
    robot_state_->setJointGroupAccelerations(jmg_, accelerations[wp_count - 2]);

    robot_trajectory.addSuffixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(*robot_state_)), timestamps_[wp_count-2]-timestamps_[wp_count-3]);

    robot_state_->setJointGroupPositions(jmg_, joint_space_path[wp_count - 1]);
    robot_state_->setJointGroupVelocities(jmg_, velocities[wp_count - 1]);
    robot_state_->setJointGroupAccelerations(jmg_, accelerations[wp_count - 1]);

    robot_trajectory.addSuffixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(*robot_state_)), timestamps_[wp_count-1]-timestamps_[wp_count-2]);

    ROS_INFO("Generated trajectory with %lu waypoints", robot_trajectory.getWayPointCount());
}
