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
 * Title:   dynamic_programming_solver.h 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 30, 2018
 *
 * The dynamic programming solver executes dynamic programming on one
 * or more state space grids (objects of the StateSpaceGrid2D class).
 * If the model constraints allow for a solution to exist, the solver
 * finds the globally optimal solution and stores it in a
 * RobotTrajectory object that is returned to the caller.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DYNAMIC_PROGRAMMING_SOLVER_H_
#define INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DYNAMIC_PROGRAMMING_SOLVER_H_

#include <moveit_dp_redundancy_resolution/state_space_grid.h>
#include <moveit_dp_redundancy_resolution/objective_function.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_dp_redundancy_resolution
{

class DynamicProgrammingSolver
{

public:

    DynamicProgrammingSolver(   const robot_model::RobotModelConstPtr & model,
                                const moveit::core::JointModelGroup * jmg,
                                const ObjectiveFunction & objective_function);

    ~DynamicProgrammingSolver();

    void addGrid(StateSpaceGrid2D * grid);
    void setTimeVector(const std::vector<double> & timestamps);

    bool solve(robot_trajectory::RobotTrajectory & robot_trajectory);

private:

    void initializeGridsForMaximization_();
    void initializeGridsForMinimization_();
    void enableAllInitialStates_();
    void computeCostsFromNode_(unsigned int stage, unsigned int rp_index, StateSpaceNode * prev, StateSpaceNode * node);
    void computeAngleVectorsDifference_(Eigen::VectorXd & diff, const Eigen::VectorXd & minuend, const Eigen::VectorXd & subtrahend) const;
    bool satisfiesAccelerationBounds_(const robot_state::RobotState * robot_state) const;
    StateSpaceNode * findLastNode_();
    void createRobotTrajectory_(robot_trajectory::RobotTrajectory & robot_trajectory, const StateSpaceNode * last_node);

    std::vector<StateSpaceGrid2D *> grids_;
    std::vector<double> timestamps_;
    const robot_model::RobotModelConstPtr model_;
    const moveit::core::JointModelGroup * jmg_;
    robot_state::RobotState * robot_state_;
    robot_state::RobotStatePtr robot_state_ptr_;

    ObjectiveFunction objective_function_;
};

}



#endif /* INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DYNAMIC_PROGRAMMING_SOLVER_H_ */
