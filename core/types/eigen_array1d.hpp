/**
 * @file eigen_array1d.hpp
 * @brief 
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_EIGEN_ARRAY1D_HPP_
#define TCPP_EIGEN_ARRAY1D_HPP_

#include "array1d.hpp"

#include <cassert>
#include <eigen3/Eigen/Core>

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Adapter class of Array1d for Eigen::Array
 */
template <typename T, int rows>
class EArray1d: public Array1d<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] earray Eigen::Array<T, rows, 1>
	 */
	EArray1d( Eigen::Array<T, rows, 1>& earray ): earray_(earray) {}

	/********** Methods **********/
	/**
	 * @brief Get the size of the array
	 * @return Size of the array
	 */
	int size() const { return earray_.rows(); }

	/**
	 * @brief Get the const reference of the element
	 * @param[in] index Index of the element
	 */
	const T& elem( int index ) const {
		assert( index >= 0 && index < size() );
		return earray_.coeff( index );
	}

	/**
	 * @brief Get the reference of the element
	 * @param[in] index Index of the element
	 */
	T& elemRef( int index ) {
		assert( index >= 0 && index < size() );
		return earray_.coeffRef( index );
	}
	
private:
	Eigen::Array<T, rows, 1>& earray_;
};

/**
 * @brief Adapter class of Array1d for Eigen::Vector
 */
template <typename T, int rows>
class EVector: public Array1d<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] earray Eigen::Array<T, rows, 1>
	 */
	EVector( Eigen::Matrix<T, rows, 1>& evector ): evector_(evector) {}

	/********** Methods **********/
	/**
	 * @brief Get the size of the array
	 * @return Size of the array
	 */
	int size() const { return evector_.rows(); }

	/**
	 * @brief Get the const reference of the element
	 * @param[in] index Index of the element
	 */
	const T& elem( int index ) const {
		assert( index >= 0 && index < size() );
		return evector_.coeff( index );
	}

	/**
	 * @brief Get the reference of the element
	 * @param[in] index Index of the element
	 */
	T& elemRef( int index ) {
		assert( index >= 0 && index < size() );
		return evector_.coeffRef( index );
	}
	
private:
	Eigen::Matrix<T, rows, 1>& evector_;
};

} /* namespace tcpp */

#endif /* TCPP_EIGEN_ARRAY1D_HPP_ */

