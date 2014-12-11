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
/** Builds Viewpoint Pose Tree **/

#include "vision_module/vtree_user.hpp"

/**
 * This file depends on the ros parameter server and expects that the following parameters are set:
 * 	<param  name="command" type="string" value="/init" />
 *	<param  name="clouds_folder" type="string" value="path/to/traindata" />
 *	<param  name="database_location" type="string" value="path/to/database" />
 *	<param  name="tree_k" type="int" value="5" />
 *  <param  name="tree_levels" type="int" value="5" />
 */
int build_tree_main(int argc, char **argv)
{
	ros::init(argc, argv, "build_tree");
	ros::NodeHandle nh;
	/*
	// For debugging purposes
	std::string command;
	node_handle.param ("command", command, std::string("/load"));
	std::cout << "command = " << command << std::endl;
	*/
	
	ros_vtree_user vt_user( nh );
	//std::cout << "tree object constructed!" << std::endl;
	
	return vt_user.start();
}

int main(int argc, char **argv)
{
	return build_tree_main( argc, argv );
}
