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
#include <misc.hpp>
#include <ros/package.h>

int
run_virtual_kinect(int argc, char ** argv)
{
	ros::init (argc, argv, "virtual_kinect");
	
	// Set scene
    std::string virtual_kinect_dir( ros::package::getPath("virtual_kinect_pkg") );
    std::string ply_file_path(virtual_kinect_dir + "/../database/haagen_dazs.ply");
    bool is_color = true;
	ros::param::get("~ply_file_path", ply_file_path );
    ros::param::get("~is_color",is_color);
		
	ROS_INFO("The ply file is set to: %s", ply_file_path.c_str());

	// Set initial pose
	double px = 1.0;
	double py = 1.0;
	double pz = 0.0;
	
	ros::param::get("~px", px );
	ros::param::get("~py", py );
	ros::param::get("~pz", pz );

	Eigen::Vector3d init_position( px, py, pz);
	Eigen::Vector4d init_orientation = misc::target2quat( init_position, Eigen::Vector3d(0.0,0.0,0.0) );

	ROS_INFO_STREAM("The initial position set to: " << init_position.transpose());
	ROS_INFO_STREAM("The initial orientation set to: " << init_orientation.transpose());
	
	// Create virtual kinect and initialize
	int coord = 1;	// 0 = obj coor, 1 = optical, 2 = standard
	bool add_noise = true;
    virtual_kinect vk( init_position, init_orientation, coord, false, add_noise,is_color);
	vk.init_vkin( ply_file_path );
	
	// Set rate
	ros::Rate loop_rate(20);
	int cnt = 0;
	srand((unsigned)time(NULL));
	while ( ros::ok() )
	{
		vk.spin_vkin();
				
		ros::spinOnce();
		loop_rate.sleep();

		//ROS_INFO("virtual_kinect running... %d",cnt);
		if( cnt % 3 == 0 )
		{
			Eigen::Vector3d np( misc::uniform_cont(-2.0, 2.0), 
						misc::uniform_cont(-2.0, 2.0), misc::uniform_cont(0.0, 1.5) );
			Eigen::Vector4d no = misc::target2quat( np, Eigen::Vector3d(0.0,0.0,0.0) );
			vk.send_goal( np, no );
		}
		cnt++;
	}
	
	return 0;
}

int
main(int argc, char ** argv)
{
	return run_virtual_kinect(argc, argv);	
}
