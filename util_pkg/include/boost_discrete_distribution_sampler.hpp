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

#pragma once

#include <ctime>
#include <boost/random/mersenne_twister.hpp>
//#include <boost/random/discrete_distribution.hpp>

// Add to avoid discrete_distribution.hpp
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
// ************




class boost_discrete_distribution_sampler
{
	
private:
	boost::mt19937 generator;
	//boost::random::discrete_distribution<> dist;
	
	// Add to avoid discrete_distribution.hpp
	std::vector<double> cumulative;
	boost::shared_ptr< boost::variate_generator<boost::mt19937&, boost::uniform_real<> > > disc_dist_ptr;
	// ************
	

public:
	boost_discrete_distribution_sampler( std::vector<double>::iterator dist_begin,
													 std::vector<double>::iterator dist_end )
		: generator( std::time(0) )
	{
		std::partial_sum(dist_begin, dist_end, std::back_inserter(cumulative));
		boost::uniform_real<> dist(0, cumulative.back());
		disc_dist_ptr.reset( new boost::variate_generator<boost::mt19937&, boost::uniform_real<> >(generator, dist) );
   }
	
	int sample()
	{
		return (std::lower_bound(cumulative.begin(), cumulative.end(), disc_dist_ptr->operator()()) - cumulative.begin());
	}
	
	/*
	boost_discrete_distribution_sampler( ){}
	boost_discrete_distribution_sampler( std::vector<double>::iterator dist_begin,
											 std::vector<double>::iterator dist_end )
		: generator( std::time(0) ),
		  dist( dist_begin, dist_end )
	{}
	
											 
	int sample()
	{
		return dist( generator );
	}
	*/
	
};
