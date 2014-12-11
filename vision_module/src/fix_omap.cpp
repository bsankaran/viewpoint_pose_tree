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
/** Fix to observation model **/

#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <io_utils.hpp>

int
fix_omap(int argc, char **argv)
{
	typedef boost::filesystem3::directory_iterator directory_iterator;
	// Get file paths
	if(argc < 3)
		throw std::runtime_error("Path to occluded oMap directory is required is second arg\n");
		
	if(argc < 2)
		throw std::runtime_error("Path to oMap directory is required is first arg\n");
	
	std::string dir_omap(argv[1]);
	std::string dir_omap_occ(argv[2]);
	
	for (directory_iterator itr(dir_omap); itr!=directory_iterator(); ++itr)
   {
   	// Get first txt file
   	std::string curr_ext(itr->path().extension().string());
   	if( curr_ext.compare(".txt") == 0 )
   	{
   		std::string curr_stem( itr->path().stem().string() );
   		std::string curr_name( itr->path().filename().string() );
   		
   		for (directory_iterator itr2(dir_omap_occ); itr2!=directory_iterator(); ++itr2)
   		{
   			std::string curr_occ_name( itr2->path().filename().string() );
   			if( curr_occ_name.compare(curr_name) == 0)
   			{
   				std::cout << curr_occ_name << std::endl;
   				
   				// Open files
   				std::ifstream f1, f2;
   				std::ofstream f3;
   				io_utils::open_in_file( f1, itr->path().string() );
   				io_utils::open_in_file( f2, itr2->path().string() );
   				io_utils::open_out_file( f3, "./" + curr_stem + "_combined.txt" );
   				
   				if( !f1 )
						throw std::runtime_error("Cannot open oMap file " + curr_stem + "\n");
   				if( !f2 )
						throw std::runtime_error("Cannot open occ file " + curr_stem + "\n");
   				if( !f3 )
						throw std::runtime_error("Cannot open out file...\n");
							
					// Combine them
					int cntr = 0;
					int vp1, vp2 = -1;
					std::string st1, st2;
					double sc1, sc2;
					
					while( (f1 >> vp1) && (f1 >> st1) && (f1 >> sc1) )
					{
						if( vp1 == cntr )
							f3 << vp1 << " " << st1 << " " << sc1 << std::endl;
						else
						{
							int cntr2 = 0;
							if(vp2 != -1)
							{
								f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
								cntr2 = 1;
							}

							while( (f2 >> vp2) && (f2 >> st2) && (f2 >> sc2) )
							{
								if(vp2 != cntr)
									break;
								
								if( cntr2 == 0 || cntr2 == 1 || cntr2 == 6 || cntr2 == 7 )
									f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
								else
								{
									f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
									f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
								}
								++cntr2;
								
								if(cntr2 == 8)
									cntr2 = 0;
							}
							
							f3 << vp1 << " " << st1 << " " << sc1 << std::endl;					
							++cntr;
						}
					}
					
					
					f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
					int cntr2 = 1;

					while( (f2 >> vp2) && (f2 >> st2) && (f2 >> sc2) )
					{						
						if( cntr2 == 0 || cntr2 == 1 || cntr2 == 6 || cntr2 == 7 )
							f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
						else
						{
							f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
							f3 << vp2 << " " << st2 << " " << sc2 << std::endl;
						}
						++cntr2;
						
						if(cntr2 == 8)
							cntr2 = 0;
					}
					
					// close the files
					f1.close();
					f2.close();
					f3.close();
   			}
   		}
   	}
  	}
  	  	

	return 0;
}

int
main(int argc, char **argv)
{
	return fix_omap(argc, argv);
}
