/**
 * @file emat2vector.hpp
 * @brief Convert Eigen::Matrix to std::vector<std::vector>
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP2_EMAT2VECTOR_HPP_
#define TCPP2_EMAT2VECTOR_HPP_

#include <vector>
#include <eigen3/Eigen/Core>

#ifdef USING_TBB
#include <tbb/blocked_range2d.h>
#endif

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Function class to convert Eigen::Matrix<T, rows, cols> to std::vector<std::vector<T> >
 */
template <typename T, int rows, int cols>
class EMat2Vector {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	EMat2Vector(): row_num_(0), col_num_(0), emat_(), std_vector_() {}
	
	/**
	 * @brief Constructor
	 */
	EMat2Vector( const Eigen::Matrix<T, rows, cols>& emat, std::vector<std::vector<T> >& std_vector ):
		row_num_(emat.rows()), col_num_(emat.cols()), emat_(emat), std_vector_(std_vector)
		{
			if( !std_vector_.empty() ) {
				std_vector_.clear();
			}
			
			std_vector_.resize( rows );
			for( int row = 0; row < rows; ++row ) {
				std_vector_[0].resize( cols );
			}
		}
	
	/********** Destructor **********/
	/**
	 * @brief Destructor
	 */
	virtual ~EMat2Vector() {}

	/********** Operators **********/
#ifdef USING_TBB
	/**
	 * @brief operator() for converting with tbb
	 * @param[in] range tbb::blocked_range<size_t>
	 */
	void operator()( const tbb::blocked_range2d<int>& range ) const {
		for( int row = range.rows().begin(); row != range.rows().end(); ++row ) {
			for( int col = range.cols().begin(); col != range.cols().end(); ++col ) {
				std_vector_[row][col] = emat_.coeff( row, col );
			}
		}
	}
#endif

	/**
	 * @brief operator() for converting without tbb
	 */
	void operator()() const {
		for( int row = 0; row < row_num_; ++row ) {
			for( int col = 0; col < col_num_; ++col ) {
				std_vector_[row][col] = emat_.coeff( row, col );
			}
		}
	}
	
private:
	int row_num_, col_num_;
	const Eigen::Matrix<T, rows, cols>& emat_;
	std::vector<std::vector<T> >& std_vector_;
};

} /* namespace tcpp */

#endif /* TCPP2_EMAT2VECTOR_HPP_ */
