/*
 * histogram.hpp
 *
 *  Created on: 2012/10/09
 *      Author: takahashi
 */

#ifndef TCPP_HISTOGRAM_HPP_
#define TCPP_HISTOGRAM_HPP_

#include "../core.hpp"

#include <vector>
#include <eigen3/Eigen/Core>

namespace tcpp {

typedef Eigen::ArrayXi Histogram;

template <typename T>
T GetDistance( const Histogram& histogram1, const Histogram& histogram2 ) {
	assert( histogram1.rows() == histogram2.rows() && histogram1.cols() == histogram2.cols() );
	double distance = 0.0;
	distance += ( ( ( ( histogram1 - histogram2 ).pow(2) ).cast<double>() ) / ( ( histogram1 + histogram2 ).cast<double>() + 1E-15 ) ).sum();
	distance /= 2.0;
	return static_cast<T>(distance);
}

template <typename T>
int GetDistanceArray(const std::vector<Histogram>& histograms1, const std::vector<Histogram>& histograms2, Eigen::Array<T, -1, -1>& distance_array) {
	int num_1 = static_cast<int>( histograms1.size() );
	int num_2 = static_cast<int>( histograms2.size() );
	distance_array = Eigen::Array<T, -1, -1>::Zero(num_1, num_2);
	for( int i1 = 0 ; i1 < num_1 ; ++i1 )
		for( int i2 = 0 ; i2 < num_2 ; ++i2 )
			distance_array.coeffRef( i1, i2 ) = GetDistance<T>( histograms1[i1], histograms2[i2] );
	return 0;
}

} /* namespace tcpp */

#endif /* TCPP_HISTOGRAM_HPP_ */
