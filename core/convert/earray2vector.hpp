/**
 * @file earray2vector.hpp
 * @brief Convert Eigen::Array(single) to std::vector
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP2_EARRAY2VECTOR_HPP_
#define TCPP2_EARRAY2VECTOR_HPP_

#include <vector>
#include <eigen3/Eigen/Core>

#ifdef USING_TBB
#include <tbb/blocked_range.h>
#endif

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Function class to convert Eigen::Array<T, size, 1> to std::vector<T>
 */
template <typename T, int size>
class EArray2Vector {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	EArray2Vector(): size_(0), earray_(), std_vector_() {}
	/**
	 * @brief Constructor
	 */
	EArray2Vector( const Eigen::Array<T, size, 1>& earray, std::vector<T>& std_vector ):
		size_(earray.rows()), earray_(earray), std_vector_(std_vector)
		{
			if( !std_vector_.empty() )
				std_vector_.clear();
			std_vector.reserve( size_ );
		}

	/********** Destructor **********/
	virtual ~EArray2Vector() {}

	/********** Operators **********/
#ifdef USING_TBB
	/**
	 * @brief operator() for converting with tbb
	 * @param[in] range tbb::blocked_range<size_t>
	 */
	void operator()( const tbb::blocked_range<int>& range ) const {
		for( int i = range.begin(); i != range.end(); ++i ) {
			std_vector_.push_back( earray_.coeff( i ) );
		}
	}
#endif
	/**
	 * @brief operator() for converting without tbb
	 */
	void operator()() const {
		for( int i = 0; i < size_; ++i ) {
			std_vector_.push_back( earray_.coeff( i ) );
		}
	}
	
private:
	int size_;
	const Eigen::Array<T, size, 1>& earray_;
	std::vector<T>& std_vector_;
};

} /* namespace tcpp */

#endif /* TCPP2_EARRAY2VECTOR_HPP_ */
