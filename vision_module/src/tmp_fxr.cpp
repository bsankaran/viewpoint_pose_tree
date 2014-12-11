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
#include <boost/filesystem.hpp>
#include <io_utils.hpp>

int
tmp_fxr(int argc, char **argv)
{
	std::ifstream f1;
	std::ofstream f2;
	io_utils::open_in_file( f1, "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/vision_module/data/omap/occ1/oMap_bigbox.txt");
	io_utils::open_out_file( f2, "/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/vision_module/data/omap/occ1/oMap_bigbox_fx.txt" );
	
						
	if( !f1 )
		throw std::runtime_error("Cannot open file 1\n");
	if( !f2 )
		throw std::runtime_error("Cannot open file 2\n");
		
	int cntr = 0;
	int vp1 = -1;
	std::string st1;
	double sc1;
	
	int dcntr = -1;
	while( (f1 >> vp1) && (f1 >> st1) && (f1 >> sc1) )
	{
		if(( vp1 > dcntr ) && (cntr < 16 ))
		{
			f2 << vp1 << " " << st1 << " " << sc1 << std::endl;
			++cntr;
		}
		
		if( cntr == 16)
		{	
			++dcntr;
			cntr = 0;
		}
	}
	f1.close();
	f2.close();
	
	return 0;
}

int
main(int argc, char **argv)
{
	return tmp_fxr(argc, argv);
}
