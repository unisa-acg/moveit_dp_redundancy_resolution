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
 * Title:   workspace_trajectory.cpp 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA - Automatic Control Group
 * Date:    Jul 20, 2018
 *
 * See workspace_trajectory.h for a description of the class.
 *
 * -------------------------------------------------------------------
 */

#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>
#include <moveit_dp_redundancy_resolution/dp_redundancy_resolution_capability.h>
#include <moveit_dp_redundancy_resolution/exceptions.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace moveit_dp_redundancy_resolution;

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const std::vector<double> & time,
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z,
    const std::vector<double> & roll,
    const std::vector<double> & pitch,
    const std::vector<double> & yaw):
            name_(name),
            timestamps_(time)
{
    length_ = time.size();

    if( x.size() != length_ || y.size() != length_ || z.size() != length_ || roll.size() != length_ || pitch.size() != length_ || yaw.size() != length_ )
        throw moveit_dp_redundancy_resolution::Exception("Trajectories for single coordinates must be of the same size");

    checkTimestamps_();

    for(int i=0; i<length_; i++)
    {
        tf::Quaternion q = tf::createQuaternionFromRPY(roll[i], pitch[i], yaw[i]);

        geometry_msgs::Pose pose;
        pose.position.x = x[i];
        pose.position.y = y[i];
        pose.position.z = z[i];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        waypoints_.push_back(pose);
    }
}

WorkspaceTrajectory::WorkspaceTrajectory(const moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg)
{
    setWorkspaceTrajectoryMsg(ws_trajectory_msg);
}

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const std::vector<geometry_msgs::Pose> & waypoints,
    const std::vector<double> & timestamps):
        name_(name),
        waypoints_(waypoints),
        timestamps_(timestamps)
{
    length_ = timestamps.size();

    if(waypoints.size() != length_)
        throw moveit_dp_redundancy_resolution::Exception("The time vector must have as many elements as the waypoints");

    checkTimestamps_();
}

/**
    \brief Constructs a WorkspaceTrajectory starting from the joint space trajectory \e jst, by using FW kinematics

    \param name. The name to be given to the generated trajectory.
    \param jst. The joint space trajectory from which to compute the task space trajectory
    \param rs. The robot state object to be used to call the FW kinematics function
    \param planning_group. The planning group whose EE corresponding to the task tip (for which to compute FW kinematics)
*/

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const moveit_msgs::RobotTrajectory & jst,
    const robot_state::RobotStatePtr rs,
    const std::string & planning_group):
        name_(name),
        length_(jst.joint_trajectory.points.size())
{
    ROS_INFO_NAMED("WorkspaceTrajectory", "Creating trajectory with %u waypoints", length_);

    const robot_state::JointModelGroup* jmg = rs->getJointModelGroup(planning_group);

    for(int i=0; i<length_; i++)
    {
        trajectory_msgs::JointTrajectoryPoint point = jst.joint_trajectory.points[i];

        rs->setJointGroupPositions(jmg, point.positions);

        const Eigen::Affine3d & ee_pose = rs->getGlobalLinkTransform(jmg->getLinkModelNames().back());

        geometry_msgs::Pose pose_msg;

        tf::poseEigenToMsg(ee_pose, pose_msg);

        waypoints_.push_back(pose_msg);

        timestamps_.push_back(point.time_from_start.toSec());
    }

    checkTimestamps_();

    ROS_INFO_NAMED("WorkspaceTrajectory", "Created trajectory '%s' with %u waypoints", name_.c_str(), length_);
}

WorkspaceTrajectory::WorkspaceTrajectory(
        const std::string & name,
        const std::string & file_path):
                name_(name)
{
    if(file_path == "" || !boost::filesystem::exists(file_path))
        throw moveit_dp_redundancy_resolution::Exception("File path is not set or the file does not exist");

    std::ifstream in_file(file_path, std::ifstream::in | std::ifstream::binary);

    in_file.read((char *)&length_, sizeof(unsigned int));

    for(int i=0; i < length_; i++)
    {
        double timestamp;
        geometry_msgs::Pose pose;

        in_file.read((char *)&timestamp, sizeof(double));
        in_file.read((char *)&(pose.position.x), sizeof(double));
        in_file.read((char *)&(pose.position.y), sizeof(double));
        in_file.read((char *)&(pose.position.z), sizeof(double));
        in_file.read((char *)&(pose.orientation.x), sizeof(double));
        in_file.read((char *)&(pose.orientation.y), sizeof(double));
        in_file.read((char *)&(pose.orientation.z), sizeof(double));
        in_file.read((char *)&(pose.orientation.w), sizeof(double));

        timestamps_.push_back(timestamp);
        waypoints_.push_back(pose);
    }

    ROS_INFO("Loaded trajectory of %d waypoints", length_);

    in_file.close();
}

const std::vector<geometry_msgs::Pose>& WorkspaceTrajectory::getWaypoints() const
{
    return waypoints_;
}

int WorkspaceTrajectory::getLength() const
{
    return length_;
}

void WorkspaceTrajectory::getWorkspaceTrajectoryMsg(moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg) const
{
    ws_trajectory_msg.name = name_;
    ws_trajectory_msg.waypoints = waypoints_;
    ws_trajectory_msg.timestamps = timestamps_;
}

void WorkspaceTrajectory::setWorkspaceTrajectoryMsg(const moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory & ws_trajectory_msg)
{
    if(ws_trajectory_msg.waypoints.size() != ws_trajectory_msg.timestamps.size())
        throw moveit_dp_redundancy_resolution::Exception("The message must have as many timestamps as waypoints");

    name_ = ws_trajectory_msg.name;
    waypoints_ = ws_trajectory_msg.waypoints;
    timestamps_ = ws_trajectory_msg.timestamps;
    length_ = ws_trajectory_msg.waypoints.size();

    checkTimestamps_();
}

void WorkspaceTrajectory::scaleLinearly(float gradient)
{
    for(int i=0; i<length_; i++)
    {
        timestamps_[i] = timestamps_[i] * gradient;
    }

    checkTimestamps_();
}

void WorkspaceTrajectory::setEvenlySpacedTimestamps(double duration)
{
    if(duration <= 0.0)
        throw moveit_dp_redundancy_resolution::Exception("Trajectory duration must be greater than zero");

    std::vector<double> time_range(length_);

    generateRange(time_range, timestamps_[0], timestamps_[0]+duration, length_);

    timestamps_ = time_range;
}

double WorkspaceTrajectory::getDuration() const
{
    if(length_ == 0)
        return 0;

    return (timestamps_[length_-1] - timestamps_[0]);
}

void WorkspaceTrajectory::exportToBinary(const std::string & file_path) const
{
    std::ofstream out_file(file_path, std::ofstream::out | std::ofstream::binary);

    if(out_file.rdstate() != std::ios_base::goodbit)
        throw moveit_dp_redundancy_resolution::Exception("Could not create the output file");

    out_file.write((char *)&length_, sizeof(unsigned int));

    for(int i=0; i < length_; i++)
    {
        out_file.write((char *)&(timestamps_[i]), sizeof(double));
        out_file.write((char *)&(waypoints_[i].position.x), sizeof(double));
        out_file.write((char *)&(waypoints_[i].position.y), sizeof(double));
        out_file.write((char *)&(waypoints_[i].position.z), sizeof(double));
        out_file.write((char *)&(waypoints_[i].orientation.x), sizeof(double));
        out_file.write((char *)&(waypoints_[i].orientation.y), sizeof(double));
        out_file.write((char *)&(waypoints_[i].orientation.z), sizeof(double));
        out_file.write((char *)&(waypoints_[i].orientation.w), sizeof(double));
    }

    out_file.close();
}

void WorkspaceTrajectory::printTimestamps() const
{
    for(int i=0; i<length_; i++)
        ROS_INFO("Index = %d, Time = %f", i, timestamps_[i]);
}

void WorkspaceTrajectory::checkTimestamps_() const
{
    if(timestamps_.size() == 0)
        throw moveit_dp_redundancy_resolution::Exception("No timestamps available");

    if(timestamps_.size() > 1)
    {
        for(int i=1; i<timestamps_.size(); i++)
        {
            if(timestamps_[i] <= timestamps_[i-1])
                throw moveit_dp_redundancy_resolution::Exception("Time is not an increasing monotone function");
        }
    }
}
