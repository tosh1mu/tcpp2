/**
 * @file algorithm.hpp
 * @brief algorithm
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP2_ALGORITHM_HPP_
#define TCPP2_ALGORITHM_HPP_

#include <vector>
#include <complex>
#include <map>
#include <stdexcept>

namespace tcpp {

template <typename KT, typename VT>
VT GetMaxVal( const std::map<KT, VT>& map, KT& max_key )
{
	int first = 1;
	VT max_val;
	for( typename std::map<KT, VT>::const_iterator itr = map.begin();
		 itr != map.end(); ++itr ) {
		KT key = itr->first;
		VT val = itr->second;

		if( first ) {
			max_key = key;
			max_val = val;
			continue;
		}

		if( val > max_val ) {
			max_key = key;
			max_val = val;
		}
	}
	return max_val;
}

// Return value is in range (period/2, period/2]
template <typename Iterator>
typename Iterator::value_type GetCyclicAverage(
		Iterator b, Iterator e,
		typename Iterator::value_type const &period, typename Iterator::value_type const &epsilon = 0)
{
	typedef typename Iterator::value_type T;

	std::complex<T> v(0, 0);
	for ( ; b != e; ++b)
	{
		T phase = (*b) * 2 * M_PI / period;
		v += std::complex<T>(std::cos(phase), std::sin(phase));
	}

	if (std::norm(v) <= epsilon) throw std::runtime_error( "Failed to calculate cyclic average" );

	return std::arg(v) * period / 2 / M_PI;
}

template <typename ValIterator, typename WIterator>
typename ValIterator::value_type GetWeightedCyclicAverage(
	ValIterator vb, ValIterator ve, WIterator wb, WIterator we,
	typename ValIterator::value_type const &period, typename ValIterator::value_type const &epsilon = 0)
{
	typedef typename ValIterator::value_type VT;
	typedef typename WIterator::value_type WT;

	std::complex<VT> v(0, 0);
	for ( ; vb != ve && wb != we; ++vb, ++wb)
	{
		VT phase = (*vb) * 2 * M_PI / period;
		v += std::complex<VT>( (*wb) * std::cos(phase), (*wb) * std::sin(phase));
	}

	if (std::norm(v) <= epsilon) throw std::runtime_error( "Failed to calculate cyclic average" );

	return std::arg(v) * period / 2 / M_PI;
}

} /* namespace tcpp */

#endif /* TCPP2_ALGORITHM_HPP_ */
