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


#include <string>
#include <stdexcept>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ros
#include <ros/package.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// boost
#include <boost/lexical_cast.hpp>

// custom
#include <io_utils.hpp>
#include <misc.hpp>

int
vpm(int argc, char **argv)
{
	if( argc < 2 )
		throw std::runtime_error("Provide model name as second argument");
	std::string obj_name(argv[1]);
	std::string vision_module_dir( ros::package::getPath("vision_module") );
	std::string tree_vps_path( vision_module_dir + "/data/tree_vps.txt");
	std::string views_path( vision_module_dir + "/../database/cloud_data/"
														   + obj_name + "_views" );
														   
														   
	// Read the tree vps file
	Eigen::MatrixXd tree_vps;
	io_utils::file2matrix( tree_vps_path, tree_vps, 3 );
	int num_vps = tree_vps.rows();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_model_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	for(int vp = 0; vp < num_vps; ++vp)
	{
		// load the view
		pcl::PointCloud<pcl::PointXYZ>::Ptr view_ptr( new pcl::PointCloud<pcl::PointXYZ>);
		std::string view_path( views_path + "/" + obj_name + "_" 
													 + boost::lexical_cast<std::string>(vp) +".pcd" );
		pcl::io::loadPCDFile<pcl::PointXYZ> ( view_path, *view_ptr );
		
		// rotate to world frame
		Eigen::Vector3d camera_position = tree_vps.row(vp).transpose();
		Eigen::Vector4d camera_orientation = misc::target2quat( camera_position, 
																				  Eigen::Vector3d(0,0,0) );
		
		tf::Transform map_T_snsr;
		map_T_snsr.setOrigin( tf::Vector3(camera_position.x(), 
													 camera_position.y(), 
													 camera_position.z()) );
		map_T_snsr.setRotation( tf::Quaternion( camera_orientation.x(), 
															 camera_orientation.y(), 
															 camera_orientation.z(), 
															 camera_orientation.w()) );
		
		tf::Transform snsr_T_optical;
		snsr_T_optical.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
		snsr_T_optical.setRotation( tf::Quaternion( -0.5, 0.5, -0.5, 0.5 ) );
		
		tf::Transform map_T_optical = map_T_snsr * snsr_T_optical;

		pcl_ros::transformPointCloud( *view_ptr, *view_ptr, map_T_optical);
		
		// add to pcd_model
		*pcd_model_ptr +=  *view_ptr;
	}
	
	// Save
	pcl::PCDWriter writer;
	writer.writeASCII( vision_module_dir + "../database/pcd_models/" + obj_name + "_complete.pcd", *pcd_model_ptr );
	
	return 0;
}

int
main(int argc, char **argv)
{
	return vpm(argc, argv);
}
