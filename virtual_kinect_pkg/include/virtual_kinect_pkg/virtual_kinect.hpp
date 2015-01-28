/*****************************************************************************************
 *
 *  Copyright (c) 2014, Bharath Sankaran,
 *  Computational Learning and Motor Control Laboratory, University of Southern California
 *  (bsankara@usc.edu)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bharath Sankaran nor CLMC
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************************/

#pragma once
// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>

// PCL_ROS
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Actionlib
#include <actionlib/server/simple_action_server.h>
#include <virtual_kinect_pkg/GoToPoseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// CUSTOM
#include "virtual_kinect_pkg/vkin_offline.hpp"


class virtual_kinect : public vkin_offline
{

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	
	// communication
	ros::Publisher pose_pub;
	ros::Publisher cloud_pub;
    ros::Publisher color_cloud_pub;

    //Color parser
    bool is_color_;
	
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }
	
	// NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	actionlib::SimpleActionServer<virtual_kinect_pkg::GoToPoseAction> as_;
	virtual_kinect_pkg::GoToPoseResult result_;
	
	actionlib::SimpleActionClient<virtual_kinect_pkg::GoToPoseAction> ac_;
	
public:
	virtual_kinect( const Eigen::Vector3d position, const Eigen::Vector4d orientation,
                    bool obj_coord, bool organized, bool add_noise, bool is_color );

	~virtual_kinect(){
		//ROS_INFO("Shutting down virtual_kinect node!");
		pose_pub.shutdown();
		cloud_pub.shutdown();
	}
	
	void spin_vkin();
	void actionCallback(const virtual_kinect_pkg::GoToPoseGoalConstPtr &goal);
	
	// For testing purposes:
	void send_goal(const Eigen::Vector3d & goal_position, 
			   	   const Eigen::Vector4d & goal_orientation);
			   	   
	void doneCb(const actionlib::SimpleClientGoalState& state,
	   const virtual_kinect_pkg::GoToPoseResultConstPtr& result);
	/**/   
private:
							 
	
	void publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
    void publish_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
	void publish_pose();
	

};

