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

// STANDARD
#include <iostream>
#include <cmath>
#include <cassert>

// VTK
#include <vtkGeneralTransform.h>
#include <vtkMath.h>

// CUSTOM
#include "virtual_kinect_pkg/vkin_offline.hpp"
#include "misc.hpp"

void 
vkin_offline::init_vkin( const std::string ply_file_path )
{
	vtkSmartPointer<vtkPolyData> scene_ = loadPLYAsDataSet( ply_file_path.c_str() );
	
	// xmin, xmax, ymin, ymax, zmin, zmax 
	scene_->GetBounds (bounds);
	
	/*
	std::cout << "The bounds are "
			  << bounds[0] << " "
			  << bounds[1] << " "
			  << bounds[2] << " "
			  << bounds[3] << " "
			  << bounds[4] << " "
			  << bounds[5] << std::endl;
	*/		  
	tree = vtkSmartPointer<vtkCellLocator>::New ();
	tree->SetDataSet (scene_);
	tree->CacheCellBoundsOn ();
	tree->SetTolerance (0.0);
	tree->SetNumberOfCellsPerBucket (1);
	tree->AutomaticOn ();
	tree->BuildLocator ();
	tree->Update ();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense()
{
	/*
	std::cout << "Sensing at position " << position_.transpose()
			  << " and orientation " << orientation_.transpose() << std::endl;
	*/		  
	return sense( position_, orientation_ );
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d & position, const Eigen::Vector3d & target )
{
	// Determine the orientation	
	Eigen::Vector4d orientation = misc::target2quat(position, target);
	
	/*	
	std::cout << "Sensing at position " << position_.transpose()
			  << " and orientation " << orientation.transpose() << std::endl;
	*/
	return sense( position, orientation );
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
vkin_offline::sense( const Eigen::Vector3d & position, const Eigen::Vector4d & orientation )
{
	double up[3] = {0.0, 0.0, 0.0};
	double right[3] = {0.0, 0.0, 0.0};
	double x_axis[3] = {1.0, 0.0, 0.0};
	double z_axis[3] = {0.0, 0.0, 1.0};
	double eye[3], viewray[3];

	// Camera position
	eye[0] = position.x();
	eye[1] = position.y();
	eye[2] = position.z();

	// Viewray, right, and up
	// In Standard coordinate system (x = forward, y = left, z = up)
	// wRs = [viewray | left | up]
	Eigen::Matrix<double,3,3> wRs = misc::quat2rot( orientation );
	
	viewray[0] = wRs(0,0);
	viewray[1] = wRs(1,0);
	viewray[2] = wRs(2,0);	
	if (fabs(viewray[0]) < EPS) viewray[0] = 0;
	if (fabs(viewray[1]) < EPS) viewray[1] = 0;
	if (fabs(viewray[2]) < EPS) viewray[2] = 0;

	right[0] = -wRs(0,1);
	right[1] = -wRs(1,1);
	right[2] = -wRs(2,1);	
	if (fabs(right[0]) < EPS) right[0] = 0;
	if (fabs(right[1]) < EPS) right[1] = 0;
	if (fabs(right[2]) < EPS) right[2] = 0;

	up[0] = wRs(0,2);
	up[1] = wRs(1,2);
	up[2] = wRs(2,2);	
	if (fabs(up[0]) < EPS) up[0] = 0;
	if (fabs(up[1]) < EPS) up[1] = 0;
	if (fabs(up[2]) < EPS) up[2] = 0;


   // Prepare the point cloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

   double temp_beam[3], beam[3], p[3];
  	double p_coords[3], x[3], t;
  	int subId;
  	
	// Create a transformation
	vtkGeneralTransform* tr1 = vtkGeneralTransform::New ();
	vtkGeneralTransform* tr2 = vtkGeneralTransform::New ();
	 
	 // Sweep vertically    
	for (double vert = sp.vert_start; vert <= sp.vert_end; vert += sp.vert_res)
	{
		
		tr1->Identity ();
		tr1->RotateWXYZ (vert, right);
		tr1->InternalTransformPoint (viewray, temp_beam);

		// Sweep horizontally
		for (double hor = sp.hor_start; hor <= sp.hor_end; hor += sp.hor_res)
		{
		
		  // Create a beam vector with (lat,long) angles (vert, hor) with the viewray
			tr2->Identity ();
			tr2->RotateWXYZ (hor, up);
			tr2->InternalTransformPoint (temp_beam, beam);
			vtkMath::Normalize (beam);

			// Find point at max range: p = eye + beam * max_dist
			for (int d = 0; d < 3; d++)
				p[d] = eye[d] + beam[d] * sp.max_dist;

			// Put p_coords into laser scan at packet id = vert, scan id = hor
			/*
			std::cout <<"eye = "<< eye[0] << " " << eye[1] << " " << eye[2] << std::endl;
			std::cout <<"p = " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			std::cout <<"x = " << x[0] << " " << x[1] << " " << x[2] << std::endl;
			std::cout <<"t = " << t << ", subId = " << subId << std::endl;
			*/

			// Determine if the ray between eye (camera position) and p (max range)
			// intersects with the tree given a tolerance of 0
			// return the intersection coordinates in x in the WORLD frame
			// return the cell which was intersected by the ray in cellId
			vtkIdType cellId;
			if (tree->IntersectWithLine (eye, p, 0, t, x, p_coords, subId, cellId))
			{ 
				// x are the coordinates in the world frame
				pcl::PointXYZ pt;
				switch( sp.coord )
				{
					case 0:{		// object coordinates
						pt.x = static_cast<float> (x[0]); 
						pt.y = static_cast<float> (x[1]); 
						pt.z = static_cast<float> (x[2]);
						break;
					}
					case 2:{
						// camera coordinates: x = forward, y = left, z = up
						// Translate the origin to the sensor position
						x[0] -= eye[0];
						x[1] -= eye[1];
						x[2] -= eye[2];	
						
						// sRw = wRs^T = [viewray ; left ; up]
						pt.x = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] );
						pt.y = static_cast<float> ( -right[0]*x[0] - right[1]*x[1] - right[2]*x[2] );
						pt.z = static_cast<float> ( up[0]*x[0] + up[1]*x[1] + up[2]*x[2] );
						break;
					}
					default:{
						// optical coordinates: z = forward, x = right, y = down
					
						// Translate the origin to the sensor position
						x[0] -= eye[0];
						x[1] -= eye[1];
						x[2] -= eye[2];
					
						pt.x = static_cast<float> ( right[0]*x[0] + right[1]*x[1] + right[2]*x[2] );
						pt.y = static_cast<float> ( -up[0]*x[0] - up[1]*x[1] - up[2]*x[2] );
						pt.z = static_cast<float> ( viewray[0]*x[0] + viewray[1]*x[1] + viewray[2]*x[2] ); 

					}
				}
			 	
			 	cloud_ptr->points.push_back (pt);
			}
			else if (sp.organized)
			{
				pcl::PointXYZ pt;
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();

				cloud_ptr->points.push_back (pt);
			 }
		} // Horizontal
	} // Vertical
	 
	// Add noise
	if(sp.add_noise)
	{
		if ( sp.coord == 0 )
	 		addNoise(position, cloud_ptr, gaussian_rng);
	 	else
	 		addNoise(Eigen::Vector3d(0.0,0.0,0.0), cloud_ptr, gaussian_rng);
	}
	 
	if (sp.organized)
	{
		cloud_ptr->height = 1 + static_cast<uint32_t> ((sp.vert_end - sp.vert_start) / sp.vert_res);
		cloud_ptr->width = 1 + static_cast<uint32_t> ((sp.hor_end - sp.hor_start) / sp.hor_res);
	}
	else
	{
		cloud_ptr->width = static_cast<uint32_t> (cloud_ptr->points.size ());
		cloud_ptr->height = 1;
	}

	return cloud_ptr;
}


void 
vkin_offline::addNoise( const Eigen::Vector3d position, 
					    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
					    vkin_offline::GEN &generator )
{		
	Eigen::Vector3d ray_vec;
	double noise_param = 0.0025;		// TRUE IS 0.0005
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
