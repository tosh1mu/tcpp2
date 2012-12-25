/**
 * @file vector2d.hpp
 * @brief 2-dimensional extension of std::vector<T>
 * @author Toshimitsu Takahashi
 * @date 2012/12/25
 *
 */

#ifndef TCPP_VECTOR2D_HPP_
#define TCPP_VECTOR2D_HPP_

#define USING_TBB

#include <vector>

#ifdef USING_TBB
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#endif

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief 2-dimensional extended class of std::vector<T>
 */
template <typename T>
class Vector2d {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	Vector2d(): vector2d_() {}

	/**
	 * @brief Copy constructor
	 */
	Vector2d( const Vector2d& vector2d ): vector2d_(vector2d) {}

	/**
	 * @brief Constructor with std::vector
	 */
	Vector2d( const std::vector<std::vector<T> >& vector2d ): vector2d_(vector2d) {}

	/********** Destructor **********/
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Vector2d() {}

	/********* Initializers **********/
	/**
	 * @brief Set all elements to 0
	 */
	void InitZero() {
		int rows = static_cast<int>( vector2d_.size() );
		if( rows > 0 ) {
			int cols = static_cast<int>( vector2d_[0].size() );
		}
	}
	/**
	 * @brief Resize to (size x size) and set all elements to 0
	 */
	void InitZero( int size ) {
		
	}
	
private:
	/********** Member Variables **********/
	std::vector<std::vector<T> >& vector2d_;

	/********** Resizers **********/
	/**
	 * @brief Base class of resizers
	 */
	class Resizer {
	public:
		/********** Constructors **********/
		/**
		 * @brief Default constructor
		 */
		Resizer(): rows_(0), cols_(0), vector2d_() {}

		/**
		 * @brief Constructor
		 */
		Resizer( std::vector<std::vector<T> >& vector2d, int rows, int cols ):
			rows_(rows), cols_(cols), vector2d_(vector2d) {}

		/********** Destructor **********/
		/**
		 * @brief Destructor
		 */
		virtual ~Resizer() {}

		/********** Operators **********/
#ifdef USING_TBB
		/**
		 * @brief operator() for tbb::parallel_for
		 * @param[in] range tbb::blocked_range<int>
		 */
		void operator()( const tbb::blocked_range<int>& range ) const {
			
		}
#endif
		
	private:
		int rows_, cols_; //!< Size parameters
		std::vector<std::vector<T> >& vector2d_; //!< Target std::vector<std::vector<T> >
	};

	/********** Initializers **********/
	/**
	 * @brief Abstract class of initializers
	 */
	class Initializer {
	public:
		/********** Constructors **********/
		/**
		 * @brief Default constructor
		 */
		Initializer():vector2d_() {}

		/**
		 * @brief Constructor
		 * @param[in] vector2d std::vector<std::vector<T> > to initialize
		 */
		Initializer( std::vector<std::vector<T> >& vector2d ): vector2d_(vector2d) {}

		/********** Destructor **********/
		/**
		 * @brief Destructor
		 */
		virtual ~Initializer() {}

		/********** Operators **********/
#ifdef USING_TBB
		/**
		 * @brief operator() for tbb::parallel_for
		 * @param[in] range tbb::blocked_range2d<int>
		 */
		virtual void operator()( const tbb::blocked_range2d<int>& range ) {}
#endif
		
		/**
		 * @brief operetor() to use in case no tbb
		 */
		virtual void operator()() {}
		
	protected:
		std::vector<std::vector<T> >& vector2d_; //!< Reference to target

		/**
		 * @brief Common function to set element values
		 * @param[in] row Row of the element
		 * @param[in] col Column of the element
		 * @param[in] val Value the element set to be
		 */
		void SetElement( int row, int col, T val ) {
			vector2d_[row][col] = val;
		}
	};

	/**
	 * @brief Initializer class to initialize all the elements with a constant
	 */
	class ConstInitializer: public Initializer {
	public:
		/********** Constructors **********/
		/**
		 * @brief Default constructor
		 */
		ConstInitializer(): Initializer(), constant_(0) {}

		/**
		 * @brief Constructor
		 * @param[in] vector2d std::vector<std::vector<T> > to initialize
		 * @param[in] constant Constant value all the elements set to be
		 */
		ConstInitializer( std::vector<std::vector<T> >& vector2d, T constant ):
			Initializer(vector2d), constant_(constant) {}

		/********** Destructor **********/
		/**
		 * @brief Destructor
		 */
		virtual ~ConstInitializer() {}

		/********** Operators **********/
#ifdef USING_TBB
		/**
		 * @brief Overloaded operator() for tbb::parallel_for
		 * @param[in] tbb::blocked_range2d<int>
		 */
		void operator()( const tbb::blocked_range2d<int>& range ) {
			for( int row = range.rows().begin(); row != range.rows().end(); ++row ) {
				for( int col = range.cols().begin(); col != range.cols().end(); ++col ) {
					SetElement( row, col, constant_ );
				}
			}
		}
#endif
		
		/**
		 * @brief Overloaded operator() to use in case no tbb
		 */
		void operator()() {
			for( int row = 0, rows = static_cast<int>(vector2d_.size()); row < rows; ++row ) {
				for( int col = 0, cols = static_cast<int>(vector2d_[0].size()); col < cols; ++col ) {
					SetElement( row, col, constant_ );
				}
			}
		}
		
	private:
		T constant_; //!< Constant value all the elements set to be
	};
};

} /* namespace tcpp */

#endif /* TCPP_VECTOR2D_HPP_ */
