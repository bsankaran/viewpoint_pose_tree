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
/** Adds noise to the input pcd file and tests output on VP-Tree **/


#include <fstream>
#include <sstream>
#include <stdexcept>
#include <ros/package.h>

#include "vision_module/vtree_user.hpp"
#include "virtual_kinect_pkg/vkin_offline.hpp"
#include <pcl/io/pcd_io.h>
#include "misc.hpp"
#include "io_utils.hpp"


void addNoise( Eigen::Vector3d const & position,
			   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			   boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > &generator)
{
	Eigen::Vector3d ray_vec;
	double noise_param = 0.005;		// TRUE IS 0.0005
	double noise_std;
	double sq_norm;
	for (std::size_t cp = 0; cp < cloud->points.size(); cp++)
	{
		// get a vector pointing from the camera to the point
		ray_vec.x() = cloud->points[cp].x - position.x();
		ray_vec.y() = cloud->points[cp].y - position.y();
		ray_vec.z() = cloud->points[cp].z - position.z();

		// normalize it
		sq_norm = ray_vec.squaredNorm();
		ray_vec = ray_vec / sqrt(sq_norm);

		// get the noise vector magnitude
		noise_std = noise_param * sq_norm;

		//set the correct size
		ray_vec = generator()*noise_std*ray_vec;

		// add the noise
		cloud->points[cp].x += ray_vec.x();
		cloud->points[cp].y += ray_vec.y();
		cloud->points[cp].z += ray_vec.z();
	}
}

int build_omap_main(int argc, char **argv)
{
	std::cout << "Starting oMap build process..." << std::endl;

	if(argc < 3)
		throw std::runtime_error("A directory to save the confMat.txt is required...\n");
	if(argc < 2)
		throw std::runtime_error("A pcd file is required as a first argument...\n");

	// Get directory paths and hypothesis id
	std::string ply_file_path( argv[1] );

	std::string save_dir_path( argv[2] );
    std::string vision_module_path(ros::package::getPath("vision_module") );

	// Form the output filename
	std::stringstream omap_file_path;
	omap_file_path << save_dir_path << "/confMatorg.txt";

	std::ofstream outfile ( omap_file_path.str().c_str(), ofstream::app );

	if(!outfile.is_open())
		throw std::runtime_error("Unable to open conf mat save file...\n");

	// Construct and start a vocabulary tree
	vtree_user vt("/load", vision_module_path + "/data", vision_module_path + "/../database/cloud_data/");
	vt.start();

	// noise
	boost::mt19937 rng(time(0));
	boost::normal_distribution<double> normal_distrib(0.0, 1.0);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gaussian_rng( rng, normal_distrib );


	pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout<<"Loading pcd file"<<std::endl;
	pcl::io::loadPCDFile<pcl::PointXYZ> (ply_file_path, *noise_cloud_ptr);
	std::cout<<"Done loading pcd file"<<std::endl;
	//add noise
	addNoise( Eigen::Vector3d(0,0,0), noise_cloud_ptr, gaussian_rng );
	std::pair<float,std::string> vp_score = vt.top_match( noise_cloud_ptr->getMatrixXfMap() );

	// append vp and vp_score to the file
	outfile << argv[1]<<" "<<argv[7] << " " << vp_score.second << " " << vp_score.first << std::endl;

	outfile.close();

	return 0;
}


int main(int argc, char **argv)
{
	return build_omap_main( argc, argv );
}




