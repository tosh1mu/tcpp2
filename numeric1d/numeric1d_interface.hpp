/**
 * @file numeric1d_interface.hpp
 * @brief Interface of 1-dimensional numeric types
 * @author Toshimitsu Takahashi
 * @date 2012/12/27
 *
 */

#ifndef TCPP_NUMERIC1D_INTERFACE_HPP_
#define TCPP_NUMERIC1D_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of 1-dimensional numeric types
 */
template <typename T>
class Numeric1dInterface {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Numeric1dInterface() = 0;

	/********** Virtual accessors **********/
	/**
	 * @brief Virtual const accessor to the coefficient
	 */
	virtual const T& coeff( int index ) const = 0;

	/**
	 * @brief Virtual accessor to the coefficient
	 */
	virtual T& coeffRef( int index ) = 0;

	/********** Virtual mutators **********/
	/**
	 * @brief Virtual mutaror of the coefficient
	 */
	virtual void set_coeff( int index, T value ) = 0;

	/********** Parameter getting functions **********/
	/**
	 * @brief Virtual function to get the number of coefficients
	 */
	virtual int count() const = 0;
};

template <typename T>
Numeric1dInterface<T>::~Numeric1dInterface() {}

} /* namespace tcpp */

#endif /* TCPP_NUMERIC1D_INTERFACE_HPP_ */
