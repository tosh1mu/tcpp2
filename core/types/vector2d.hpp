/**
 * @file vector2d.hpp
 * @brief 2-dimensional extension of std::vector<T>
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_VECTOR2D_HPP_
#define TCPP_VECTOR2D_HPP_

#include "array2d.hpp"
#include <vector>

#ifdef USING_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief 2-dimensional extended class of std::vector<T>
 */
template <typename T>
class Vector2d: public Array2d<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	Vector2d(): container_() {}

	/**
	 * @brief Copy constructor
	 */
	Vector2d( const Vector2d& vector2d ): container_(vector2d.container()) {}

	/**
	 * @brief Constructor with std::vector
	 */
	explicit Vector2d( const std::vector<std::vector<T> >& container ): container_(container) {}

	/********** Destructor **********/
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Vector2d() = 0;

	/********** Accessor **********/
	/**
	 * @brief Get const reference of the core container
	 * @return Const reference of the core container (std::vector<std::vector<T> >)
	 */
	const std::vector<std::vector<T> >& container() { return container_; }

	/**
	 * @brief Get const reference of the element
	 */
	const T& elem( int row, int col ) const {
		assert( row >= 0 && row < rows() && col >= 0 && col < cols() );
		return container_[row][col];
	}

	/**
	 * @brief Get reference of the element
	 */
	T& elemRef( int row, int col ) const {
		assert( row >= 0 && row < rows() && col >= 0 && col < cols() );
		return container_[row][col];
	}

	/********* Initializing methods **********/
	/**
	 * @brief Initialize and set all the element to zero
	 * @param[in] rows Number of rows
	 * @param[in] cols Number of columns
	 */
	void InitZero( int rows, int cols ) {
		Initializer zero_initializer( container_, rows, cols, 0 );
#ifdef USING_TBB
		tbb::blocked_range<int> range(0, rows, 100);
		tbb::parallel_for( range, zero_initializer );
#else
		zero_initializer();
#endif
	}

	/********** Property getting methods **********/
	/**
	 * @brief Get the number of rows
	 */
	int rows() const { return static_cast<int>( container_.size() ); }

	/**
	 * @brief Get the number of columns
	 */
	int cols() const {
		if( container_.size() > 0 ) {
			return static_cast<int>( container_.size() );
		} else {
			return -1;
		}
	}

	/**
	 * @brief Get the number of elements
	 */
	int size() const {
		if( container_.size() > 0 ) {
			return rows() * cols();
		} else {
			return -1;
		}
	}

	/**
	 * @brief Confirm whether the container is empty
	 */
	bool empty() const {
		return container_.empty();
	}

private:
	/********** Member Variables **********/
	std::vector<std::vector<T> > container_;

	/********** Member Classes **********/
	/**
	 * @brief Initializer
	 */
	class Initializer {
	public:
		/********** Constructors **********/
		/**
		 * @brief Constructor
		 * @param[in] vector2d std::vector<std::vector<T> > to initialize
		 */
		Initializer( std::vector<std::vector<T> >& container, int rows, int cols, T constant ):
			container_(container), rows_(rows), cols_(cols), constant_(constant) {}

		/********** Operators **********/
#ifdef USING_TBB
		/**
		 * @brief virtual operator() for tbb::parallel_for
		 * @param[in] range tbb::blocked_range<int>
		 */
		void operator()( const tbb::blocked_range<int>& range ) const {
			for( int row = range.begin(); row != range.end(); ++row ) {
				container_.push_back( std::vector<T>( cols_, constant_ ) );
			}
		}
#endif
		
		/**
		 * @brief virtual operetor() to use in case no tbb
		 */
		void operator()() const {
			for( int row = 0; row < rows_; ++row ) {
				container_.push_back( std::vector<T>( cols_, constant_ ) );
			}
		}
		
	protected:
		std::vector<std::vector<T> >& container_; //!< Reference to target container
		int rows_, cols_; //!< Size parameters
		T constant_; //!< Constant
	};
};

} /* namespace tcpp */

#endif /* TCPP_VECTOR2D_HPP_ */
