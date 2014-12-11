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
#include "boost_discrete_distribution_sampler.hpp"

int test_dd_samp(int argc, char **argv)
{
	const double prob_a[] = {0.125,0.125,0.0625,0.0625,0.125,0.125,0.0625,0.0625,0.125,0.125};
	std::vector<double> probs_v( prob_a, prob_a + sizeof(prob_a) / sizeof(prob_a[0]) );
	
	const int nrolls = 10000; // number of experiments
  	const int nstars = 100;   // maximum number of stars to distribute
	
	boost_discrete_distribution_sampler bdds( probs_v.begin(), probs_v.end() );
	
	int p[10]={};

	for (int i=0; i<nrolls; ++i) 
	{
   	int number = bdds.sample();
   	++p[number];
  	}

  	std::cout << "a discrete_distribution:" << std::endl;
  	for (int i=0; i<10; ++i)
   	std::cout << i << ": " << std::string(p[i]*nstars/nrolls,'*') << std::endl;
	
	return 0;
}

int main(int argc, char **argv)
{
	return test_dd_samp( argc, argv );
}
