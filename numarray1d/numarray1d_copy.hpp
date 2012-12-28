/**
 * @file numarray1d_copy.hpp
 * @brief Definition of copy operation of 1-dimensional numeric arrays
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_NUMARRAY1D_COPY_HPP_
#define TCPP_NUMARRAY1D_COPY_HPP_

#include "numarray1d_interface.hpp"
#include <cassert>

#ifdef USING_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif /* USING_TBB */

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Method class to copy tcpp::NumArray1d
 */
template <typename T1, typename T2>
class CopyNumArray1d {
public:
	/**
	 * @brief Constructor
	 */
	CopyNumArray1d( const NumArray1dInterface<T1>& src_array1d, NumArray1dInterface<T2>& dst_array1d ):
		src_array1d_(src_array1d), dst_array1d_(dst_array1d) {}

#ifdef USING_TBB
	/**
	 * @brief Operator for tbb::parallel_for
	 */
	void operator()( const tbb::blocked_range<int>& range ) const {
		for( int ind = range.begin(); ind != range.end(); ++ind ) {
			dst_array1d_.set_coeff( ind, static_cast<T2>( src_array1d_.coeff(ind) ) );
		}
	}
#endif /* USING_TBB */

	/**
	 * @brief Operator to use in case no tbb
	 */
	void operator()() const {
		for( int ind = 0; ind < src_array1d_.length(); ++ind ) {
			dst_array1d_.set_coeff( ind, static_cast<T2>( src_array1d_.coeff(ind) ) );
		}
	}

private:
	const NumArray1dInterface<T1>& src_array1d_;
	NumArray1dInterface<T2>& dst_array1d_;
};

template <typename T1, typename T2>
void Copy( const NumArray1dInterface<T1>& src_array1d, NumArray1dInterface<T2>& dst_array1d ) {
	assert( src_array1d.length() == dst_array1d.length() );
	CopyNumArray1d<T1, T2> copy( src_array1d, dst_array1d );
#ifdef USING_TBB
	tbb::blocked_range<int> range( 0, src_array1d.length(), 100 );
	tbb::parallel_for( range, copy );
#else /* USING_TBB */
	copy();
#endif /* USING_TBB */
}

} /* namespace tcpp */

#endif /* TCPP_NUMARRAY1D_COPY_HPP_ */
