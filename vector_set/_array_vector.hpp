/*
 * array_vector.hpp
 *
 *  Created on: 2012/11/02
 *      Author: takahashi
 */

#ifndef TCPP_ARRAY_VECTOR_HPP_
#define TCPP_ARRAY_VECTOR_HPP_

#include "../core.hpp"

#include <vector>
#include <boost/array.hpp>

namespace tcpp {

template <typename T, unsigned int dimension>
class ArrayVector {
private:
	/* typedef */
	typedef boost::array<T, dimension> Array;
	
public:
	/* constructor */
	
	/* accessor */
	const std::vector<Array>& arrays() const { return arrays_; }
	
	/* mutator */
	void extend( const ArrayVector& array_vector ) {
		for(unsigned int i=0 ; i<array_vector.size() ; ++i)
			arrays_.push_back(array_vector[i]);
	}
	
	/* method */
	
public:		/* std::vector */
	/* constructor */
	ArrayVector(): arrays_() {}
	ArrayVector( size_t size ): arrays_(size) {}
	ArrayVector( size_t num, const Array& val ): arrays_( num, val ) {}
	ArrayVector( const ArrayVector& from ): arrays_( from.arrays() ) {}
//	ArrayVector( std::vector::input_iterator start, std::vector::input_iterator end ): arrays_(start, end) {}

	/* operator */
	const Array& operator[]( size_t index ) const { return arrays_[index]; }

	/* accessor */
	const Array& at( size_t index ) const { return arrays_.at(index); }
	bool empty() const { return arrays_.empty(); }
	size_t size() const { return arrays_.size(); }

	/* mutator */
	void push_back( const Array& array ) { arrays_.push_back(array); }
	void reserve( size_t n ) { arrays_.reserve(n); }
	void resize( size_t n ) { arrays_.resize(n); }
	void resize( size_t n, const Array& array ) { arrays_.resize(n, array); }
	void clear() { arrays_.clear(); }

	/* destructor */
	virtual ~ArrayVector() {}

private:
	std::vector<Array> arrays_;
};

} /* namespace tcpp */

#endif /* TCPP_ARRAY_VECTOR_HPP_ */
