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

#include "virtual_kinect_pkg/virtual_kinect.hpp"


virtual_kinect::virtual_kinect( const Eigen::Vector3d position, 
								const Eigen::Vector4d orientation, bool obj_coord, 
								bool organized, bool add_noise )
	: vkin_offline( position, orientation, obj_coord, organized, add_noise ),
	  private_nh_("~"), action_name_("go_to_pose_act"),
	  as_(nh_, "go_to_pose_act", boost::bind(&virtual_kinect::actionCallback, this, _1), false),
	  ac_("go_to_pose_act", true)
{
	// Start the action server
	as_.start();
	
	pose_pub = nh_.advertise<geometry_msgs::PoseStamped> ( "vkinect_pose", 1 );
	cloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "vkinect_cloud", 1 );
}

void 
virtual_kinect::spin_vkin()
{	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld = sense();	

	publish_pose();
	publish_cloud(cld);

}


void 
virtual_kinect::publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
	cloud_ptr -> header.frame_id = "/sensor_optical";
	cloud_ptr -> header.stamp = ros::Time::now(); //ros::Time(0);
	ROS_INFO("Cloud has %d points", static_cast<unsigned int>(cloud_ptr->size()));
	cloud_pub.publish(*cloud_ptr);

}	



void 
virtual_kinect::publish_pose()
{	
	ros::Time curr_time = ros::Time::now();

	// broadcast the transform
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position_.x(), position_.y(), position_.z()) );
	transform.setRotation( tf::Quaternion( orientation_.x(), orientation_.y(), 
														orientation_.z(), orientation_.w()) );
	
	// Broadcast a sensor to world transform
	br.sendTransform(tf::StampedTransform(transform, curr_time, "/map", "/sensor"));
	
	// Broadcast an optical sensor to sensor transform, which converts
	// FORWARD: Z -> X; RIGHT: X -> -Y; DOWN: Y -> -Z
	transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
	transform.setRotation( tf::Quaternion( -0.5, 0.5, -0.5, 0.5 ) );
 	br.sendTransform(tf::StampedTransform(transform, curr_time, "/sensor", "/sensor_optical"));

	// Send the kinect pose
	geometry_msgs::PoseStamped pose;
	
	pose.pose.position.x = position_.x();
	pose.pose.position.y = position_.y();
	pose.pose.position.z = position_.z();
	pose.pose.orientation.x = orientation_.x();
	pose.pose.orientation.y = orientation_.y();
	pose.pose.orientation.z = orientation_.z();
	pose.pose.orientation.w = orientation_.w();
		
	pose.header.frame_id = "/map";
	pose.header.stamp = ros::Time::now();
	pose_pub.publish(pose);
}

void 
virtual_kinect::actionCallback(const virtual_kinect_pkg::GoToPoseGoalConstPtr &goal)
{
	//ROS_INFO("Goal received...");
	// Simulate movement here
	position_.x() = goal->goal_pose_arr[0].pose.position.x;
	position_.y() = goal->goal_pose_arr[0].pose.position.y;
	position_.z() = goal->goal_pose_arr[0].pose.position.z;

	orientation_.x() = goal->goal_pose_arr[0].pose.orientation.x;
	orientation_.y() = goal->goal_pose_arr[0].pose.orientation.y;
	orientation_.z() = goal->goal_pose_arr[0].pose.orientation.z;
	orientation_.w() = goal->goal_pose_arr[0].pose.orientation.w;
	
	bool success = true;
	
	if(success)
	{
		result_.success = success;
		as_.setSucceeded(result_);
	}
}

// For Testing purposes
void 
virtual_kinect::send_goal(const Eigen::Vector3d & goal_position, 
			   			  const Eigen::Vector4d & goal_orientation)
{
	ROS_INFO("Sending goal...");
	virtual_kinect_pkg::GoToPoseGoal goal;
	geometry_msgs::PoseStamped goal_pose;
	
	goal_pose.pose.position.x = goal_position.x();
	goal_pose.pose.position.y = goal_position.y();
	goal_pose.pose.position.z = goal_position.z();
	
	goal_pose.pose.orientation.x = goal_orientation.x();
	goal_pose.pose.orientation.y = goal_orientation.y();
	goal_pose.pose.orientation.z = goal_orientation.z();
	goal_pose.pose.orientation.w = goal_orientation.w();

	goal.goal_pose_arr.push_back(goal_pose);
	
	ac_.sendGoal(goal, boost::bind(&virtual_kinect::doneCb, this, _1, _2), actionlib::SimpleActionClient<virtual_kinect_pkg::GoToPoseAction>::SimpleActiveCallback(), actionlib::SimpleActionClient<virtual_kinect_pkg::GoToPoseAction>::SimpleFeedbackCallback());
	
}

void 
virtual_kinect::doneCb(const actionlib::SimpleClientGoalState& state,
	   const virtual_kinect_pkg::GoToPoseResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
}
/**/










