/**
 * @file evector.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_EVECTOR_HPP_
#define TCPP_EVECTOR_HPP_

#include "numarray1d_interface.hpp"
#include <cassert>
#include <eigen3/Eigen/Core>

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Adapter class of Eigen::Matrix<T, rows, 1> for Numeric1dInterface<T>
 */
template <typename T, int rows>
class EVector: public NumArray1dInterface<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] body Main body(Eigen::Array<T, rows, 1>)
	 */
	EVector( Eigen::Matrix<T, rows, 1>& body ): body_(body) {}

	/********** Accessors **********/
	/**
	 * @brief Access to the main body
	 */
	const Eigen::Matrix<T, rows, 1>& body() const { return body_; }

	/**
	 * @brief Access to the main body
	 */
	Eigen::Matrix<T, rows, 1>& bodyRef() const { return body_; }

	/**
	 * @brief Access to the coefficient
	 * @param[in] index Index of the coefficient
	 * @return const reference of the coefficient
	 */
	const T& coeff( int index ) const {
		assert( index >= 0 && index < length() );
		return body_.coeff(index);
	}

	/**
	 * @brief Access to the coefficient
	 * @param[in] index Index of the coefficient
	 * @return reference (non-const) of the coefficient
	 */
	T& coeffRef( int index ) {
		assert( index >= 0 && index < length() );
		return body_.coeffRef(index);
	}

	/********** Mutators **********/
	/**
	 * @brief Set the coefficient to the specified value
	 * @param[in] index Index of the coefficient
	 * @param[in] value Value the coefficient is set to
	 */
	void set_coeff( int index, T value ) {
		assert( index >= 0 && index < length() );
		body_.coeffRef(index) = value;
	}

	/**
	 * @brief Set the coefficient to the specified value
	 * @param[in] index Index of the coefficient
	 * @param[in] value Value the coefficient is set to
	 */
	template <typename T1>
	void SetCoeff( int index, T1 value ) {
		assert( index >= 0 && index < length() );
		body_.coeffRef(index) = static_cast<T>( value );
	}

	/**
	 * @brief Get the number of coefficients
	 */
	int length() const {
		return body_.rows();
	}

private:
	Eigen::Matrix<T, rows, 1>& body_;
};

} /* namespace tcpp */

#endif /* TCPP_EVECTOR_HPP_ */
