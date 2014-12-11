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

#include <iostream>
#include <stdexcept>
#include <vector>

#include "io_utils.hpp"

void
io_utils::file2matrix(const std::string &file, Eigen::MatrixXd &mat, int cols )
{
	std::ifstream in;

	open_in_file( in, file );

	if(!in){
		std::cout << "[io_utils] Cannot open file: " << file << std::endl;
		throw std::runtime_error("Runtime error...\n");
	}
	
	std::vector<double> data;
	double val;
	while ( in >> val ){
		data.push_back( val );
	}

	// copy the vector to an eigen matrix
	int rows = data.size() / cols;
	mat.resize( rows, cols );
	std::copy( data.data(), data.data()+data.size(), mat.data() );
	
	for(int r = 0; r < rows; ++r)
		for(int c = 0; c < cols; ++c)
		{
			mat(r,c) = data[cols*r + c];
		} 

	in.close();
}

int io_utils_test(int argc, char **argv)
{
	return 0;
}

/*
int main(int argc, char **argv)
{
	return io_utils_test(argc, argv);
}
*/
