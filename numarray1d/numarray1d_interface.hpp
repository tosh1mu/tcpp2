/**
 * @file numarray1d_interface.hpp
 * @brief Interface of 1-dimensional numeric arrays
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_NUMARRAY1D_INTERFACE_HPP_
#define TCPP_NUMARRAY1D_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of 1-dimensional numeric types
 */
template <typename T>
class NumArray1dInterface {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~NumArray1dInterface() {}

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
	virtual int length() const = 0;
};

} /* namespace tcpp */

#endif /* TCPP_NUMARRAY1D_INTERFACE_HPP_ */
