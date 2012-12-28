/**
 * @file stl_vector2d.hpp
 * @brief 2-dimensional extension of std::vector
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_STL_VECTOR2D_HPP_
#define TCPP_STL_VECTOR2D_HPP_

#include "numarray2d_interface.hpp"
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
class StlVector2d: public NumArray2dInterface<T> {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	StlVector2d(): container_() {}

	/**
	 * @brief Copy constructor
	 */
	StlVector2d( const StlVector2d& vector2d ): container_(vector2d.container()) {}

	/**
	 * @brief Constructor with std::vector
	 */
	explicit StlVector2d( const std::vector<std::vector<T> >& container ): container_(container) {}

	/********** Destructor **********/
	/**
	 * @brief Virtual destructor
	 */
	virtual ~StlVector2d() = 0;

	/********** Accessor **********/
	/**
	 * @brief Get const reference of the core container
	 * @return Const reference of the core container (std::vector<std::vector<T> >)
	 */
	const std::vector<std::vector<T> >& container() const { return container_; }

	/**
	 * @brief Get reference of the core container
	 * @return Const reference of the core container (std::vector<std::vector<T> >)
	 */
	std::vector<std::vector<T> >& containerRef() { return container_; }

	/**
	 * @brief Get const reference of the coefficient
	 */
	const T& coeff( int row, int col ) const {
		assert( row >= 0 && row < rows() && col >= 0 && col < cols() );
		return container_[row][col];
	}

	/**
	 * @brief Get reference of the coefficient
	 */
	T& coeffRef( int row, int col ) const {
		assert( row >= 0 && row < rows() && col >= 0 && col < cols() );
		return container_[row][col];
	}

	/********* Initializing methods **********/
	/**
	 * @brief Initialize and set all the element to the constant
	 * @param[in] rows Number of rows
	 * @param[in] cols Number of columns
	 */
	template <typename T1>
	void InitConst( int rows, int cols, T1 constant ) {
		Initializer initializer( *this, rows, cols, static_cast<T>(constant) );
#ifdef USING_TBB
		tbb::blocked_range<int> range( 0, rows, 100 );
		tbb::parallel_for( range, initializer );
#else
		initializer();
#endif
	}

	/**
	 * @brief Initialize and set all the element to zero
	 * @param[in] rows Number of rows
	 * @param[in] cols Number of columns
	 */
	void InitZero( int rows, int cols ) {
		InitConst( rows, cols, 0 );
	}

	/**
	 * @brief Initialize and set all the element to zero
	 * @param[in] rows Number of rows
	 * @param[in] cols Number of columns
	 */
	void InitOne( int rows, int cols ) {
		InitConst( rows, cols, 1 );
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
			return static_cast<int>( container_[0].size() );
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
		Initializer( StlVector2d<T>& stl_vector2d, int rows, int cols, T constant ):
			stl_vector2d_(stl_vector2d), rows_(rows), cols_(cols), constant_(constant) {}

		/********** Operators **********/
#ifdef USING_TBB
		/**
		 * @brief virtual operator() for tbb::parallel_for
		 * @param[in] range tbb::blocked_range<int>
		 */
		void operator()( const tbb::blocked_range<int>& range ) const {
			for( int row = range.begin(); row != range.end(); ++row ) {
				stl_vector2d_.container_.push_back( std::vector<T>( cols_, constant_ ) );
			}
		}
#endif
		
		/**
		 * @brief virtual operetor() to use in case no tbb
		 */
		void operator()() const {
			for( int row = 0; row < rows_; ++row ) {
				stl_vector2d_.container_.push_back( std::vector<T>( cols_, constant_ ) );
			}
		}
		
	private:
		StlVector2d<T>& stl_vector2d_; //!< Reference to target container
		int rows_, cols_; //!< Size parameters
		T constant_; //!< Constant
	};
};

} /* namespace tcpp */

#endif /* TCPP_VECTOR2D_HPP_ */
