/**
 * @file container1d.hpp
 * @brief Interface of 1-dimentional containers
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_CONTAINER1D_HPP_
#define TCPP_CONTAINER1D_HPP_

#include <cassert>
#include <vector>
#include <eigen3/Eigen/Core>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T>
class Container1d {
public:
	virtual ~Container1d() = 0;
	virtual int size() = 0;
	virtual T& elem( int index ) = 0;
};

template <typename T>
class STLVector1d: public Container1d<T> {
public:
	STLVector1d() {}
	STLVector1d( std::vector<T>& stl_vector ): stl_vector_(stl_vector) {}
	int size() { return static_cast<int>( stl_vector_.size() ); }
	T& elem( int index ) {
		assert( index >= 0 && index < size() );
		return stl_vector_[index];
	}
private:
	std::vector<T>& stl_vector_;
};

template <typename T, int rows>
class EigenArray1d: public Container1d<T> {
public:
	EigenArray1d() {}
	EigenArray1d( Eigen::Array<T, rows, 1>& eigen_array ): eigen_array_(eigen_array) {}
	int size() { return eigen_array_.rows(); }
	T& elem( int index ) {
		assert( index >= 0 && index < size() );
		return eigen_array_.coeffRef( index );
	}
private:
	Eigen::Array<T, rows, 1>& eigen_array_;
};

template <typename T, int rows>
class EigenVector1d: public Container1d<T> {
public:
	EigenVector1d() {}
	EigenVector1d( Eigen::Matrix<T, rows, 1>& eigen_vector ): eigen_vector_(eigen_vector) {}
	int size() { return eigen_vector_.rows(); }
	T& elem( int index ) {
		assert( index >= 0 && index < size() );
		return eigen_vector_.coeffRef( index );
	}
private:
	Eigen::Matrix<T, rows, 1>& eigen_vector_;
};

} /* namespace tcpp */

#endif
