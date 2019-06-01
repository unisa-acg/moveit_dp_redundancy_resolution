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
 * Title:   dp_redundancy_resolution_capability.h 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 20, 2018
 *
 * The dp_redundancy_resolution_capability is a move_group plugin
 * capability providing a new planning scheme. Assuming that a
 * workspace trajectory is available, this module is capable of
 * solving the redundancy through optimization of performance indices.
 * It also publishes the solution messages to the respective topics
 * and stores them to a bag file. The published messages feed the
 * assessment applications, mainly for 3D visualization/simulation and
 * and 2D plot visualization of joints positions, velocities and
 * accelerations.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DP_REDUNDANCY_RESOLUTION_CAPABILITY_H_
#define INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DP_REDUNDANCY_RESOLUTION_CAPABILITY_H_

#include <moveit/move_group/move_group_capability.h>
#include <moveit_dp_redundancy_resolution_msgs/GetOptimizedJointsTrajectory.h>
#include <moveit_dp_redundancy_resolution/objective_function.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/assign/list_of.hpp>

namespace move_group
{

static const std::string DP_REDUNDANCY_RESOLUTION_SERVICE_NAME = "solve_redundancy_with_dp";

class MoveGroupDPRedundancyResolutionService : public MoveGroupCapability
{
    enum RobotType
    {
        planar      = 2,
        spherical   = 2,
        regional    = 4,
        spatial     = 16
    };

    static const std::string PLOT_PATH_TOPIC;
    static const std::map<std::string, RobotType> ROBOT_TYPE_MAP;

public:
    MoveGroupDPRedundancyResolutionService();
    ~MoveGroupDPRedundancyResolutionService();

    virtual void initialize();

private:
    bool computeService_(moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryRequest & req,
                         moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & res);

    bool computeOptimalSolutionPlanar_( const std::vector<double> & rp_vector,
                                        robot_trajectory::RobotTrajectory & robot_trajectory_out);

    bool computeOptimalSolutionSpatial_(const std::vector<double> & rp_vector,
                                        robot_trajectory::RobotTrajectory & robot_trajectory_out);

    bool checkRedundancyParameters_();

    moveit_msgs::DisplayTrajectory publishTrajectoryForDisplay_(  const robot_trajectory::RobotTrajectory & trajectory,
                                                                  const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & response);

    void publishTrajectoryForPlot_( const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & response);

    void publishTrajectoryToBag_(   const moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse & response,
                                    const moveit_msgs::DisplayTrajectory & disp_traj_msg,
                                    const std::string & file_path_base);

    std::string nowStr_();

    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryRequest * req_;
    moveit_dp_redundancy_resolution_msgs::GetOptimizedJointsTrajectoryResponse * res_;

    moveit_dp_redundancy_resolution::ObjectiveFunction * objective_function_;

    std::string file_path_grids_base_;
    std::string file_path_maps_base_;
    std::string file_path_bags_base_;

    ros::ServiceServer cartesian_path_service_;
    ros::Publisher display_path_;
    ros::Publisher plot_path_;
    bool plot_computed_paths_;
};

}

namespace moveit_dp_redundancy_resolution
{

static void generateRange(std::vector<double> & range, double lb, double ub, unsigned int samples)
{
    double step = (ub-lb)/(samples-1);
    int i = 0;

    while(lb <= ub)
    {
        range[i++] = lb;
        lb += step;
    }

    if(i < samples)
    {
        range[i] = ub;
    }
}

}

#endif /* INCLUDE_MOVEIT_DP_REDUNDANCY_RESOLUTION_DP_REDUNDANCY_RESOLUTION_CAPABILITY_H_ */
