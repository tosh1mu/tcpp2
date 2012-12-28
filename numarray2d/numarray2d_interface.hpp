/**
 * @file numarray2d_interface.hpp
 * @brief Interface of 2-dimensional numeric arrays
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_NUMARRAY2D_INTERFACE_HPP_
#define TCPP_NUMARRAY2D_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of 2-dimensional numeric types
 */
template <typename T>
class NumArray2dInterface {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~NumArray2dInterface() {}

	/********** Virtual accessors **********/
	/**
	 * @brief Virtual const accessor to the coefficient
	 */
	virtual const T& coeff( int row, int col ) const = 0;

	/**
	 * @brief Virtual accessor to the coefficient
	 */
	virtual T& coeffRef( int row, int col ) = 0;

	/********** Virtual mutators **********/
	/**
	 * @brief Virtual mutaror of the coefficient
	 */
	virtual void set_coeff( int row, int col, T value ) = 0;

	/********** Parameter getting functions **********/
	/**
	 * @brief Virtual function to get the number of rows
	 */
	virtual int rows() const = 0;

	/**
	 * @brief Virtual function to get the number of columns
	 */
	virtual int cols() const = 0;

	/**
	 * @brief Virtual function to get the number of coefficients
	 */
	virtual int size() const = 0;
};

} /* namespace tcpp */

#endif /* TCPP_NUMARRAY1D_INTERFACE_HPP_ */
