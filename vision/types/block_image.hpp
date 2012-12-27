/**
 * @file block_image.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2012/12/27
 *
 */

#ifndef TCPP_BLOCK_IMAGE_HPP_
#define TCPP_BLOCK_IMAGE_HPP_

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>

#ifdef USING_TBB
#include <tbb/blocked_range2d.h>
#endif

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

/**
 * @brief mean block image class
 */
template <typename T>
class BlockImage {
public:
	/**
	 * @brief Constructor
	 * @param[in] block_size
	 */
	BlockImage( const cv::Size2i& block_size ): block_size_(block_size) {}

	/**
	 * @brief Calculate means of blocks
	 * @param[in] src_image Source image
	 */
	void Init( const cv::Mat& src_image ) {}

	/**
	 * @brief Get mean block image
	 */
	void get_image( cv::Mat& block_image );

	/**
	 * @brief Get const reference of mean array
	 */
	const Eigen::Array<T, -1, -1>& mean_array() { return mean_array_; }

private:
	cv::Size2i block_size_;
	Eigen::Array<T, -1, -1> mean_array_;

	class CalcMean {
	public:
		/**
		 * @brief Constructor
		 */
		CalcMean( const cv::Mat& src_image, const cv::Size2i& block_size, Eigen::Array<T, -1, -1>& mean_array ):
			src_image_(src_image), block_size_(block_size), mean_array_(mean_array),
			row_block_num_(src_image.cols / block_size.width + 1), col_block_num_(src_image.rows / block_size.height + 1){}

#ifdef USING_TBB
		/**
		 * @brief operator() for parallel calculating
		 * @param[in] range tbb::blocked_range2d<int>
		 */
		void operator()( const tbb::blocked_range2d<int>& range ) {
		}
#endif

	private:
		const cv::Mat& src_image_;
		const cv::Size2i& block_size_;
		Eigen::Array<T, -1, -1>& mean_array_;
		int row_block_num_, col_block_num_;

		/**
		 * @brief Get ROI(cv::Rect_<int>) of the block
		 */
		void GetBlockROI( int block_row, int block_col, cv::Rect_<int>& roi_rect ) {
			assert( block_row >= 0 && block_row < row_block_num_ && block_col >= 0 && col_block_num_ );
			roi_rect.x = block_col * block_size_.width;
			roi_rect.y = block_row * block_size_.width;

			if( block_row < row_block_num_ - 1 ) {
			}

			if( block_row < row_block_num_ - 1 ) {
			}
		}
	};
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_BLOCK_IMAGE_HPP_ */
