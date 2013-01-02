/**
 * @file vector_set_interface.hpp
 * @brief Interface of vector set types
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_VECTOR_SET_INTERFACE_HPP_
#define TCPP_VECTOR_SET_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Interface class of vector set types
 */
template <typename T>
class VectorSetInterface {
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~VectorSetInterface() {}

	/**
	 * @brief
	 */
	int dimension() {}

	/**
	 * @brief
	 */
	int size() {}

	/**
	 * @brief
	 */
	
};

} /* namespace tcpp */

#endif /* TCPP_VECTOR_SET_INTERFACE_HPP_ */
