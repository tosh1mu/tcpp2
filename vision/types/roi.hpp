/**
 * @file roi.hpp
 * @brief ROI class
 * @author Toshimitsu Takahashi
 * @date 2012/12/24
 *
 */

#ifndef TCPP_ROI_HPP_
#define TCPP_ROI_HPP_

#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

/**
 * @brief ROI class
 */
template <typename T>
class ROI {
public:
	/********** Constructors **********/
	/**
	 * @brief Default constructor
	 */
	ROI(): x0_(0), y0_(0), x1_(0), y1_(0) {}
	
	/**
	 * @brief Copy constructor
	 * @param[in] roi
	 */
	ROI( const ROI<T>& roi ): x0_(roi.x0_), y0_(roi.y0_), x1_(roi.x1_), y1_(roi.y1_) {}
	
	/**
	 * @brief Constructor
	 * @param[in] x0 x-coordinate of left-upper point
	 * @param[in] y0 y-coordinate of left-upper point
	 * @param[in] x1 x-coordinate of right-lower point
	 * @param[in] y1 y-coordinate of right-lower point
	 */
	template<typename T1, typename T2, typename T3, typename T4>
	ROI( T1 x0, T2 y0, T3 x1, T4 y1 ): x0_(x0), y0_(y0), x1_(x1), y1_(y1) {}

	/********** Destructor **********/
	/**
	 * @brief Destructor
	 */
	virtual ~ROI() {}

	/********** getter **********/
	T x0() const { return x0_; }
	T y0() const { return y0_; }
	T x1() const { return x1_; }
	T y1() const { return y1_; }

	/********** setter **********/
	void set_x0( T x0 ) { x0_ = x0; }
	void set_y0( T y0 ) { y0_ = y0; }
	void set_x1( T x1 ) { x1_ = x1; }
	void set_y1( T y1 ) { y1_ = y1; }

	/********** Get properties **********/
	/**
	 * @brief check if ROI is valid
	 * @return bool(valid or invalid)
	 */
	bool IsValid() const { return (x0_<=x1_ && y0_<=y1_); }

	/**
	 * @brief get width
	 * @return width
	 */
	T width() const { return x1_ - x0_ + 1; }

	/**
	 * @brief get height
	 * @return height
	 */
	T height() const { return y1_ - y0_ + 1; }

	/********** Get OpenCV objects **********/
	/**
	 * @brief get cv::Rect_<T> of ROI
	 * @return cv::Rect_<T>
	 */
	cv::Rect_<T> cv_rect() const { return cv::Rect( x0_, y0_, width(), height() ); }
	
private:
	T x0_; //!< x-coordinate of left-upper point
	T y0_; //!< y-coordinate of left-upper point
	T x1_; //!< x-coordinate of right-lower point
	T y1_; //!< y-coordinate of right-lower point
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_ROI_HPP_ */
