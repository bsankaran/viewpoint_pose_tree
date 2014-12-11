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
/** Test Classification of VP-Tree **/

// Standard
#include <iostream>
#include <fstream>
#include <string>

// Ros
#include <ros/package.h>

// Pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

// Boost
//#include <boost/lexical_cast.hpp>

// Custom
#include "io_utils.hpp"
#include "vision_module/vtree_user.hpp"


void show_help(char *cmd_name)
{
	std::cout << "Usage " << cmd_name << " [options] file.pcd" << std::endl;
	std::cout << " * where options are:" << std::endl;
	std::cout << "     -sp path : (save_path) place the extracted pcd file in the vocabulary tree and record the matches in the file given in path" << std::endl;
	std::cout << std::endl;
}

int score_pcd_main(int argc, char **argv)
{
	if ( (pcl::console::find_switch (argc, argv, "-h")) || (argc < 2) )
	{
		show_help (argv[0]);
		exit (0);
	}
	
	// parse the command line
	std::vector<int> pcd_idx = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	std::string pcd_file_path( argv[pcd_idx[0]] );
	
	std::string score_save_path("./");
	pcl::console::parse_argument (argc, argv, "-sp", score_save_path);
	
	
	// Load the pcd file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path.c_str(), *cld_ptr);

	// Create and initialize an nbv tree
	std::string vision_module_path(ros::package::getPath("vision_module"));
	vtree_user vt("/load", vision_module_path + "/data", vision_module_path + "/../database/cloud_data");
	vt.start();
	
	// Get the score list
	ROS_INFO("Getting matches...");
	std::vector<std::pair<float,std::string> > match_names;
	vt.match_list( cld_ptr->getMatrixXfMap(), match_names, 100 );
		
	// write to file
	std::ofstream outstr;
	io_utils::open_out_file( outstr, score_save_path );

	if( !outstr )
		throw std::runtime_error("Cannot open scores save file...\n");

	for( size_t i = 0; i < match_names.size(); ++i)
	{
		outstr << match_names[i].first << " " << match_names[i].second << std::endl;
		//outstr << cluster_match_names[i].first << " " << cluster_match_names[i].second << std::endl;
	}

	outstr.close();
	
	return 0;
}


int main(int argc, char **argv)
{
	return score_pcd_main( argc, argv);
}
