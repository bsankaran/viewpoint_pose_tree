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

#include "pcd_utils.hpp"


void 
pcd_utils::color_pointcloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr,
					   pcl::PointIndices::Ptr indices,
					   Eigen::Vector3f color )
{
	for(std::vector<int>::iterator it = indices->indices.begin();
		it != indices->indices.end(); ++it)
	{
		cld_ptr->points[ (*it) ].r = color.x();
		cld_ptr->points[ (*it) ].g = color.y();
		cld_ptr->points[ (*it) ].b = color.z();
	}
}
	
int
pcd_utils::cluster_points( ANNpointArray points, int points_count, std::vector<int>& membership,
						   float r_max, float r_min, float A, float K )
{
	// adaptive radius calculation 
	// (see paper Fast and Robust Object Detection in Household Environments  
	// Using Vocabulary Trees with SIFT Descriptors by Pangercic and Haltakov)
	double radius = (1 - logsig((points_count - A) * K)) * (r_max - r_min) + r_min;
	ANNidxArray nnIdx = new ANNidx[points_count];
	ANNkd_tree* kdTree = new ANNkd_tree(points, points_count, 2);
	membership.assign(points_count, -1);
	int last_unassigned_id = 0;
	int current_cluster = 0;
  
	while (last_unassigned_id < points_count) 
	{
		std::vector<int> points_stack;
		points_stack.push_back(last_unassigned_id);
		while (points_stack.size() > 0) 
		{
			int current_point_id = points_stack.back();
			points_stack.pop_back();
			membership[current_point_id] = current_cluster;
			int points_found = kdTree->annkFRSearch(points[current_point_id],
		                                      radius, points_count, nnIdx);

			int newPointsCount = 0;
			for (int i = 0; i < points_found; ++i)
				if (membership[nnIdx[i]] == -1)
					++newPointsCount;
	  
			if (newPointsCount > 3) 
	  		{
				for (int i = 0; i < points_found; ++i)
					if (membership[nnIdx[i]] == -1)
						points_stack.push_back(nnIdx[i]);
			}
		}

		++current_cluster;
		++last_unassigned_id;
		while (last_unassigned_id < points_count && membership[last_unassigned_id] != -1)
			++last_unassigned_id;
	}
  
	delete[] nnIdx;
	delete kdTree;
	annClose();
	return current_cluster;
}


					    
int pcd_utils_test(int argc, char **argv)
{
	return 0;
}

/*
int main(int argc, char **argv)
{
	return pcd_utils_test(argc, argv);
}
*/
