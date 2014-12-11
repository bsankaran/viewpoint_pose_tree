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

#include "misc.hpp"

Eigen::Matrix<double,3,3>
misc::rotx(double roll)
{
	Eigen::Matrix<double,3,3> Rx;
	Rx << 1, 0, 0,
		  0, cos(roll), -sin(roll),
		  0, sin(roll), cos(roll);
		  
	return Rx;
}

    
Eigen::Matrix<double,3,3>
misc::roty(double pitch)
{
	Eigen::Matrix<double,3,3> Ry;
	Ry << cos(pitch), 0, sin(pitch),
		  0, 1, 0,
		  -sin(pitch), 0, cos(pitch);
	return Ry;
}


Eigen::Matrix<double,3,3>
misc::rotz(double yaw)
{
	Eigen::Matrix<double,3,3> Rz;
	Rz << cos(yaw), -sin(yaw), 0,
		  sin(yaw), cos(yaw), 0,
		  0, 0, 1;

	return Rz;
}

// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!
Eigen::Matrix<double,3,3>
misc::quat2rot( Eigen::Vector4d q )
{
	Eigen::Matrix<double,3,3> R;
	R << 1.0-2.0*(q(1)*q(1)+q(2)*q(2)), 
		  2.0*q(0)*q(1)-2.0*q(3)*q(2),
		  2.0*q(3)*q(1)+2.0*q(0)*q(2),
    	  2.0*q(0)*q(1)+2.0*q(3)*q(2),
		  1.0-2.0*(q(0)*q(0)+q(2)*q(2)),
		  2.0*q(1)*q(2)-2.0*q(3)*q(0),
    	  2.0*q(0)*q(2)-2.0*q(3)*q(1),
		  2.0*q(1)*q(2)+2.0*q(3)*q(0),
		  1.0-2.0*(q(0)*q(0)+q(1)*q(1));
	
	return R;
}

void
misc::update_belief(std::vector<double> const & prior,
					int vp,
					std::vector<int> const & vp_hist,
					Eigen::VectorXf const & scores,
					Eigen::MatrixXf const & views,
					array3D const & oMap,			//const double *oMap,
					std::vector<double> & posterior )
{
    /*
     * oMap shape
     */
    //oMap[num_obs][num_sta][num_hid];
	//const int num_obs = oMap.shape()[0];	// 21
	//const int num_sta = oMap.shape()[1];	// 15
	const int num_hid = oMap.shape()[2];	// = prior.size() = 7 = number of hypotheses
	
	     
	     
	Eigen::Matrix<float,1,1> dim_min;
	dim_min << -0.0250f;
	
	Eigen::Matrix<float,1,1> res;
	res << 0.05f;
	
	// convert the scores to cells
	Eigen::VectorXi scores_cell(scores.size());
	meters2cells(scores.transpose(),scores_cell.transpose(),dim_min,res);

	// get alpha
	double a = get_alpha(vp, vp_hist, views);
	
	// calculate the denuminator
	double denum = 0;
	

	//int tmp_idx;
	for(int h = 0; h < num_hid; ++h){
		denum += prior[h] * oMap[scores_cell[h]][vp][h];
	}
	
	// update the belief
	for(int h = 0; h < num_hid; ++h){
		posterior[h] = ((1-a) * oMap[scores_cell[h]][vp][h] / denum + a) * prior[h];
	}

}


double 
misc::great_circle_dist(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d cntr)
{
	// Center points
	a = a - cntr;
	b = b - cntr;
	
	double rad = a.norm();
	double cos_alpha = std::max(std::min(static_cast<double>((a/rad).dot(b/rad)),1.0),-1.0);
	
	return rad*std::acos(cos_alpha);
	
}


double 
misc::SO3_metric( const Eigen::Vector4d & q1, const Eigen::Vector4d & q2 ) 
{
	double quat_inner_prod = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() +q1.w()*q2.w();
	return std::acos(2*quat_inner_prod*quat_inner_prod-1);
}



