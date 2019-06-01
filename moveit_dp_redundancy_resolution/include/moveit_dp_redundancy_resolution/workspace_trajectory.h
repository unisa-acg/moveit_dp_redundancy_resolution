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
 * Title:   workspace_trajectory.h 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 20, 2018
 *
 * This class is twin to the WorkspaceTrajectory message and is aimed
 * at offering greater functionalities to generate and manipulate
 * trajectories prior to be transformed to messages to be exchanged
 * between nodes.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_WORKSPACE_TRAJECTORY_H_
#define INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_WORKSPACE_TRAJECTORY_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_dp_redundancy_resolution_msgs/WorkspaceTrajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_dp_redundancy_resolution
{

class WorkspaceTrajectory
{

public:

    WorkspaceTrajectory(const std::string & name,
                        const std::vector<double> & time,
                        const std::vector<double> & x,
                        const std::vector<double> & y,
                        const std::vector<double> & z,
                        const std::vector<double> & roll,
                        const std::vector<double> & pitch,
                        const std::vector<double> & yaw);

    WorkspaceTrajectory(const moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg);

    WorkspaceTrajectory(const std::string & name,
                        const std::vector<geometry_msgs::Pose> & waypoints,
                        const std::vector<double> & timestamps);

    WorkspaceTrajectory(const std::string & name,
                        const moveit_msgs::RobotTrajectory & joint_space_trajectory,
                        const robot_state::RobotStatePtr robot_state,
                        const std::string & planning_group);

    WorkspaceTrajectory(const std::string & name,
                        const std::string & file_path);

    ~WorkspaceTrajectory() {};

    const std::vector<geometry_msgs::Pose>& getWaypoints() const;
    int getLength() const;

    void getWorkspaceTrajectoryMsg(moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg) const;
    void setWorkspaceTrajectoryMsg(const moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg);

    void scaleLinearly(float gradient);
    void setEvenlySpacedTimestamps(double duration);
    double getDuration() const;

    void exportToBinary(const std::string & file_path) const;
    void printTimestamps() const;

private:

    void checkTimestamps_() const;

    std::string name_;
    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<double> timestamps_;
    unsigned int length_;
};

}

#endif /* INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_WORKSPACE_TRAJECTORY_H_ */
