/**
 * @file vector_set.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2013/1/2
 *
 */

#ifndef TCPP_VECTOR_SET_HPP_
#define TCPP_VECTOR_SET_HPP_

#include <cassert>
#include <vector>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T>
class VectorSet {
public:
	typedef std::vector<std::vector<T> > Body;
	VectorSet( int dimension ): body_(), dimension_(dimension) {}
	const Body& body() { return body_; }
private:
	Body body_;
	int dimension_;
};

} /* namespace tcpp */

#endif
