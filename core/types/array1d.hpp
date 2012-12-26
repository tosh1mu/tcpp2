/**
 * @file array1d.hpp
 * @brief Interface of 1-dimentional arrays
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_ARRAY1D_HPP_
#define TCPP_ARRAY1D_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of 1-dimensional array
 */
template <typename T>
class Array1d {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Array1d() = 0;

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
	virtual T& elemRef( int index ) const = 0;
};

} /* namespace tcpp */

#endif /* TCPP_ARRAY1D_HPP_ */
