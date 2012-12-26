/**
 * @file array2d.hpp
 * @brief Interface of 2-dimentional arrays
 * @author Toshimitsu Takahashi
 * @date 2012/12/26
 *
 */

#ifndef TCPP_ARRAY2D_HPP_
#define TCPP_ARRAY2D_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of 2-dimensional array
 */
template <typename T>
class Array2d {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Array2d() = 0;

	/**
	 * @brief Virtual function to get the number of rows
	 */
	virtual int rows() const = 0;

	/**
	 * @brief Virtual function to get the number of columns
	 */
	virtual int cols() const = 0;

	/**
	 * @brief Virtual function to get array size
	 */
	virtual int size() const = 0;

	/**
	 * @brief Virtual function to get const reference of the element
	 */
	virtual const T& elem( int index ) const = 0;

	/**
	 * @brief Virtual function to get reference of the element
	 */
	virtual T& elemRef( int index ) = 0;
};

} /* namespace tcpp */

#endif /* TCPP_ARRAY2D_HPP_ */
