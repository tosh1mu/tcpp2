/**
 * @file stl_array1d.hpp
 * @brief 
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_STL_ARRAY1D_HPP_
#define TCPP_STL_ARRAY1D_HPP_

#include "array1d.hpp"

#include <cassert>
#include <vector>
#include <map>

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Adapter class of Array1d for std::vector
 */
template <typename T>
class STLVector1d: public Array1d<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] stl_vector std::vector<T>
	 */
	STLVector1d( std::vector<T>& stl_vector ): stl_vector_(stl_vector) {}

	/********** Methods **********/
	/**
	 * @brief Get the size of the array
	 * @return Size of the array
	 */
	int size() const { return static_cast<int>( stl_vector_.size() ); }

	/**
	 * @brief Get the const reference of the element
	 * @param[in] Index of the element
	 */
	const T& elem( int index ) const {
		assert( index >= 0 && index < size() );
		return stl_vector_[index];
	}

	/**
	 * @brief Get the reference of the element
	 * @param[in] Index of the element
	 */
	T& elemRef( int index ) {
		assert( index >= 0 && index < size() );
		return stl_vector_[index];
	}
private:
	std::vector<T>& stl_vector_;
};

/**
 * @brief Adapter class of Array1d for std::map
 */
template <typename T>
class STLMap1d: public Array1d<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] stl_vector std::vector<T>
	 */
	STLMap1d( std::map<int, T>& stl_map ): stl_map_(stl_map) {}

	/********** Methods **********/
	/**
	 * @brief Get the size of the array
	 * @return Size of the array (Max index key of the std::map)
	 */
	int size() const {
		int max_key = -1;
		for( typename std::map<int, T>::const_iterator itr = stl_map_.begin();
			 itr != stl_map_.end(); ++itr ) {
			int key = itr->first;
			if( max_key < key )
				max_key = key;
		}
		return max_key;
	}

	/**
	 * @brief Get the const reference of the element
	 * @param[in] Index of the element
	 */
	const T& elem( int index ) const {
		assert( index >= 0 && index < size() );
		return stl_map_[index];
	}

	/**
	 * @brief Get the const reference of the element
	 * @param[in] Index of the element
	 */
	T& elemRef( int index ) {
		assert( index >= 0 && index < size() );
		return stl_map_[index];
	}
private:
	std::map<int, T>& stl_map_;
};

} /* namespace tcpp */

#endif /* TCPP_STL_CONTAINERS_HPP_ */
