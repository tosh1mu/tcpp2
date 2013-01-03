/**
 * @file lbp_extractor.hpp
 * @brief 
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 */

#ifndef TCPP_LBP_EXTRACTOR_HPP_
#define TCPP_LBP_EXTRACTOR_HPP_

#include "feature_extractor_interface.hpp"

#include <cmath>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp{
/**
 * @namespace ip
 */
namespace vision{

template <typename T>
class LBPExtractor: public FeatureExtractorInterface<T> {
public:
	/**
	 * @brief Default constructor
	 */
	LBPExtractor(): cellimg_rows_(0), cellimg_cols_(0), cell_rows_(0), cell_cols_(0) {}

	/**
	 * @brief Constructor
	 * @param cellimg_rows Number of rows
	 * @param cellimg_cols Number of cols
	 */
	LBPExtractor( size_t cellimg_rows, size_t cellimg_cols ):
		cellimg_rows_(cellimg_rows), cellimg_cols_(cellimg_cols), cell_rows_(0), cell_cols_(0),
		dimension_( cellimg_rows * cellimg_cols * pow(2, 8) ) {}

	void Extract( const cv::Mat& image, std::vector<T>& features ) {
		/* assertion */
		assert( features.empty() );

		/* calculate size of cell */
		cell_rows_ = image.rows / cellimg_rows_;
		cell_cols_ = image.cols / cellimg_cols_;

		features.resize( dimension_, 0 );
		for( Index pixel_row = 0; pixel_row < static_cast<size_t>(image.rows); ++pixel_row ) {
			for( Index pixel_col = 0; pixel_col < static_cast<size_t>(image.cols); ++pixel_col ) {
				Index cell_index = GetCellIndex( pixel_row, pixel_col );
				Index first_bin_index = cell_index * pow(2, 8);
				Index bin_index = GetBinIndex( image, pixel_row, pixel_col );
				if( bin_index < pow(2, 8) ) {
					features.at(first_bin_index + bin_index) += 1;
				}
			}
		}

	}
private:
	/* typedefs */
	typedef unsigned int Index;

	/* methods */
	/**
	 * @brief Get index of cell including specified pixel
	 * @param[in] pixel_row Index of row of the pixel
	 * @param[in] pixel_col Index of column of the pixel
	 * @return Index of cell
	 */
	Index GetCellIndex( const Index& pixel_row, const Index& pixel_col ) const {
		assert( cell_rows_ > 0 && cell_cols_ > 0 );
		Index cell_col = pixel_col / cell_cols_;
		Index cell_row = pixel_row / cell_rows_;

		if( cell_col >= cellimg_cols_ )
			--cell_col;
		if( cell_row >= cellimg_rows_ )
			--cell_row;

		return cell_row * cellimg_cols_ + cell_col;
	}

	Index GetBinIndex( const cv::Mat& image, const Index& pixel_row, const Index& pixel_col ) const {
		assert( pixel_row >= 0 && static_cast<int>(pixel_row) < image.rows && pixel_col >= 0 && static_cast<int>(pixel_col) < image.cols );
		/* check image type */
		assert( image.depth() == CV_8U );
		assert( image.channels() == 1 );
		if( pixel_row == 0 || static_cast<int>(pixel_row) == image.rows - 1 || pixel_col == 0 || static_cast<int>(pixel_col) == image.cols - 1 ) {
			return pow(2, 8);
		}
		double center_brightness = cv::saturate_cast<double>( image.at<uchar>(pixel_row, pixel_col) );
		Index bin_index = 0;
		for( Index i = 0; i < 8; ++i ) {
			Index neighbor_row, neighbor_col;
			if( i < 3 ) {
				neighbor_row = pixel_row - 1;
				neighbor_col = pixel_col - (1 - i);
			} else if( i < 5 ) {
				neighbor_row = pixel_row;
				neighbor_col = pixel_col - (7 - i*2);
			} else {
				neighbor_row = pixel_row + 1;
				neighbor_col = pixel_col - (6 - i);
			}

			double neighbor_brightness = cv::saturate_cast<double>( image.at<uchar>(neighbor_row, neighbor_col) );
			if( neighbor_brightness >= center_brightness )
				bin_index += pow(2, i);
		}
		return bin_index;
	}

	/* parameters */
	size_t cellimg_rows_; //!< Number of cells in a column of a cell-divided image
	size_t cellimg_cols_; //!< Number of cells in a row of a cell-divided image
	size_t dimension_;

	/* variables */
	size_t cell_rows_; //!< Number of pixels in a column of a cell
	size_t cell_cols_; //!< Number of pixels in a row of a cell

};


} /* namespace vision */
} /* namespace tcpp */

#endif /* LBP_EXTRACTOR_HPP_ */
