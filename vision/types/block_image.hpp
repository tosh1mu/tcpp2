/**
 * @file block_image.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2012/12/27
 *
 */

#ifndef TCPP_BLOCK_IMAGE_HPP_
#define TCPP_BLOCK_IMAGE_HPP_

#include <map>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>

#ifdef USING_TBB
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
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
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 * @param[in] block_size
	 */
	BlockImage( int block_width, int block_height ): block_size_( block_width, block_height ) {}

	/********** Initializers **********/
	/**
	 * @brief Calculate means of blocks
	 * @param[in] src_image Source image
	 */
	void Init( const cv::Mat& src_image ) {
		assert( src_image.type() == CV_8UC1 );
		block_image_ = cv::Mat::ones( src_image.size(), CV_8UC1 );
		CalcMeans calc_means( *this, src_image );
		MakeBlockImage make_block_image( *this );
#ifdef USING_TBB
		tbb::blocked_range2d<int> range( 0, calc_means.row_block_num(), 100, 0, calc_means.col_block_num(), 100 );
		tbb::parallel_for( range, calc_means );
		tbb::parallel_for( range, make_block_image );
#else
		calc_means();
		make_block_image();
#endif
	}

	/********** Accessors **********/
	/**
	 * @brief Get const reference of mean array
	 */
	const Eigen::Array<T, -1, -1>& mean_array() { return mean_array_; }

	/**
	 * @brief Get mean block image
	 */
	const cv::Mat& operator()() { return block_image_; }

private:
	cv::Size2i block_size_;
	Eigen::Array<T, -1, -1> mean_array_;
	std::map<std::pair<int, int>, cv::Rect_<int> > block_rects_;
	cv::Mat block_image_;

	/********** Method classes **********/
	/**
	 * @brief Method class to calculate mean brightnesses of the given image
	 */
	class CalcMeans {
	public:
		/**
		 * @brief Constructor
		 */
		CalcMeans( BlockImage& block_image, const cv::Mat& src_image ):
			block_image_( block_image ),
			src_image_( src_image ),
			row_block_num_( src_image.rows / block_image_.block_size_.height + 1 ),
			col_block_num_( src_image.cols / block_image_.block_size_.width + 1 )
			{
				block_image_.mean_array_ = Eigen::Array<T, -1, -1>::Zero( row_block_num_, col_block_num_ );
			}

		/**
		 * @brief Get the number of rows of block image
		 * @return Number of rows of block image
		 */
		int row_block_num() { return row_block_num_; }

		/**
		 * @brief Get the number of columns of block image
		 * @return Number of columns of block image
		 */
		int col_block_num() { return col_block_num_; }

#ifdef USING_TBB
		/**
		 * @brief operator() for tbb::parallel_for()
		 * @param[in] range tbb::blocked_range2d<int>
		 */
		void operator()( const tbb::blocked_range2d<int>& range ) const {
			for( int block_row = range.rows().begin(); block_row != range.rows().end(); ++block_row ) {
				for( int block_col = range.cols().begin(); block_col != range.cols().end(); ++block_col ) {
					cv::Rect_<int> block_rect;
					GetBlockROI( block_row, block_col, block_rect );
					block_image_.mean_array_.coeffRef( block_row, block_col ) = GetMeanBrightness( block_rect );
					block_image_.block_rects_[std::pair<int, int>( block_row, block_col )] = block_rect;
				}
			}
		}
#endif

		/**
		 * @brief operator() to use in case no tbb
		 */
		void operator()() const {
			for( int block_row = 0; block_row < row_block_num_; ++block_row ) {
				for( int block_col = 0; block_col < col_block_num_; ++block_col ) {
					cv::Rect_<int> block_rect;
					GetBlockROI( block_row, block_col, block_rect );
					block_image_.mean_array_.coeffRef( block_row, block_col ) = GetMeanBrightness( block_rect );
					block_image_.block_rects_[std::pair<int, int>( block_row, block_col )] = block_rect;
				}
			}
		}

	private:
		BlockImage& block_image_;
		const cv::Mat& src_image_;
		int row_block_num_, col_block_num_;

		/********** Methods **********/
		/**
		 * @brief Get ROI(cv::Rect_<int>) of the block
		 */
		void GetBlockROI( int block_row, int block_col, cv::Rect_<int>& block_rect ) const {
			assert( block_row >= 0 && block_row < row_block_num_ && block_col >= 0 && col_block_num_ );
			block_rect.x = block_col * block_image_.block_size_.width;
			block_rect.y = block_row * block_image_.block_size_.width;

			if( block_row < row_block_num_ - 1 ) {
				block_rect.height = block_image_.block_size_.height;
			} else {
				block_rect.height = src_image_.rows - block_image_.block_size_.height * ( row_block_num_ - 1 );
			}

			if( block_col < col_block_num_ - 1 ) {
				block_rect.width = block_image_.block_size_.width;
			} else {
				block_rect.width = src_image_.cols - block_image_.block_size_.width * ( col_block_num_ - 1 );
			}
		}

		/**
		 * @brief Get the mean brightness of the block
		 */
		T GetMeanBrightness( const cv::Rect_<int>& block_rect ) const {
			T mean_brightness = 0.0;
			for( int row = block_rect.y; row < block_rect.y + block_rect.height; ++row ) {
				for( int col = block_rect.x; col < block_rect.x + block_rect.width; ++col ) {
					mean_brightness += cv::saturate_cast<T>( src_image_.at<uchar>( row, col ) );
				}
			}
			mean_brightness /= static_cast<T>( block_rect.width * block_rect.height );
			return mean_brightness;
		}
	};

	/**
	 * @brief Method class to make mean block image
	 */
	class MakeBlockImage {
	public:
		/********** Constructor **********/
		/**
		 * @brief Constructor
		 */
		MakeBlockImage( BlockImage& block_image ): block_image_(block_image) {}

		/********** Operators **********/
#ifdef USING_TBB
		void operator()( const tbb::blocked_range2d<int>& range ) const {
			for( int block_row = range.rows().begin(); block_row != range.rows().end(); ++block_row ) {
				for( int block_col = range.cols().begin(); block_col != range.cols().end(); ++block_col ) {
					typename std::map<std::pair<int, int>, cv::Rect_<int> >::const_iterator block_itr
						= block_image_.block_rects_.find( std::pair<int, int>(block_row, block_col));
					const cv::Rect_<int>& block_rect = block_itr->second;
					DrawBlock( block_rect, block_image_.mean_array_.coeff( block_row, block_col ) );
				}
			}
		}
#endif

		void operator()() const {
			for( int block_row = 0; block_row < block_image_.mean_array_.rows(); ++block_row ) {
				for( int block_col = 0; block_col < block_image_.mean_array_.cols(); ++block_col ) {
					typename std::map<std::pair<int, int>, cv::Rect_<int> >::const_iterator block_itr
						= block_image_.block_rects_.find( std::pair<int, int>(block_row, block_col));
					const cv::Rect_<int>& block_rect = block_itr->second;
					DrawBlock( block_rect, block_image_.mean_array_.coeff( block_row, block_col ) );
				}
			}
		}

	private:
		BlockImage& block_image_;
		/********** Methods **********/
		/**
		 * @brief Set the brightness of all the pixels in the block
		 */
		void DrawBlock( const cv::Rect_<int>& block_rect, T value ) const {
			block_image_.block_image_( block_rect ) *= cv::saturate_cast<uchar>( value );
		}
	};
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_BLOCK_IMAGE_HPP_ */
