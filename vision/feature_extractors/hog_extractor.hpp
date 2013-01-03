/**
 * @file hog_extractor.hpp
 * @brief HOGExtractor class
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 */

#ifndef TCPP_HOG_EXTRACTOR_HPP_
#define TCPP_HOG_EXTRACTOR_HPP_

#include "feature_extractor_interface.hpp"

#include <cmath>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp{
/**
 * @namespace vision
 */
namespace vision{

/**
 * @class	HOGExtractor
 * @brief	HOG Extractor class
 */
template <typename T>
class HOGExtractor: public FeatureExtractorInterface<T> {
public:
	/**
	 * @brief	Default constructor
	 */
	HOGExtractor(): cellimg_rows_(0), cellimg_cols_(0),
					block_rows_(0), block_cols_(0),
					grad_orientation_num_(0),
					dimension_(0),
					blockimg_rows_(0), blockimg_cols_(0), block_num_(0), unit_angle_(0), cell_rows_(0), cell_cols_(0) {}

	/**
	 * @brief Constructor with feature parameters
	 * @param[in] cellimg_rows Number of cells in a column of a cell-divided image
	 * @param[in] cellimg_cols Number of cells in a row of a cell-divided image
	 * @param[in] block_rows Number of cells in a column of a block
	 * @param[in] block_cols Number of cells in a row of a block
	 * @param[in] grad_orientation_num Number of quantized gradient orientation
	 */
	HOGExtractor( size_t cellimg_rows, size_t cellimg_cols, size_t block_rows, size_t block_cols, size_t grad_orientation_num ):
		cellimg_rows_(cellimg_rows), cellimg_cols_(cellimg_cols), block_rows_(block_rows), block_cols_(block_cols), grad_orientation_num_(grad_orientation_num),
		dimension_( grad_orientation_num * block_rows * block_cols * ( cellimg_rows - block_rows + 1 ) * ( cellimg_cols - block_cols + 1 ) ),
		blockimg_rows_( cellimg_rows_ - block_rows_ + 1 ), blockimg_cols_( cellimg_cols_ - block_cols_ + 1 ), block_num_( blockimg_rows_ * blockimg_cols_ ),
		unit_angle_( M_PI / grad_orientation_num_ ), cell_rows_(0), cell_cols_(0)
		{
#ifdef _OPENMP
		std::cout << "OPENMP enabled. HOG features will be extracted using OPENMP." << std::endl;
#endif
		}

	size_t dimension() const { return dimension_; }

	/**
	 * @brief Extract HOG features
	 * @param[in] image Input image
	 * @param[out] features HOG features
	 */
	void Extract( const cv::Mat& image, std::vector<T>& features ) {
		/* calculate size of cell */
		cell_rows_ = image.rows / cellimg_rows_;
		cell_cols_ = image.cols / cellimg_cols_;

		/* extraction */
		CellHistograms cell_histograms = cv::Mat_<double>::zeros( cellimg_rows_*cellimg_cols_, grad_orientation_num_ );
		features.resize( dimension_, 0 );
#ifdef _OPENMP
#pragma omp parallel
#endif
		{
			Index pixel_row, pixel_col;
#ifdef _OPENMP
#pragma omp for private(pixel_col)
#endif
			for( pixel_row = 0; pixel_row < static_cast<size_t>(image.rows); ++pixel_row ) {
				for( pixel_col = 0; pixel_col < static_cast<size_t>(image.cols); ++pixel_col ) {
					Vote( image, pixel_row, pixel_col, cell_histograms );
				}
			}
#ifdef _OPENMP
#pragma omp for
#endif
			Index block;
			for( block = 0; block < block_num_; ++block ) {
				CalcBlockDescriptors( cell_histograms, block, features );
			}
		}
	}

private:
	/* typedefs */
	typedef unsigned int Index;
	typedef cv::Mat_<double> CellHistograms;

	/* struct */
	/**
	 * @struct	Gradient
	 * @brief	Parameters of gradient
	 */
	struct Gradient { double magnitude; int orientation; };

	/* static variables */
	static const double epsilon_ = 1E-10; //!< maximum number to treat as zero

	/* parameters */
	size_t cellimg_rows_; //!< Number of cells in a column of a cell-divided image
	size_t cellimg_cols_; //!< Number of cells in a row of a cell-divided image
	size_t block_rows_; //!< Number of cells in a column of a block
	size_t block_cols_; //!< Number of cells in a row of a block
	size_t grad_orientation_num_; //!< Number of quantized gradient orientation
	size_t dimension_;

	/* variables */
	size_t blockimg_rows_;
	size_t blockimg_cols_;
	size_t block_num_; //!< Number of blocks
	double unit_angle_; //!< Unit of angle to quantize
	size_t cell_rows_; //!< Number of pixels in a column of a cell
	size_t cell_cols_; //!< Number of pixels in a row of a cell

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

	/**
	 * @brief Get index list of cells in the block
	 * @param block_index
	 * @param index_list
	 */
	void GetCellIndexList( const Index& block_index, std::vector<Index>& index_list ) const {
		assert( block_index >= 0 && block_index < block_num_ );
		assert( index_list.empty() );
		index_list.reserve( block_rows_*block_cols_ );

		for( Index cell_row = block_index / blockimg_cols_; cell_row < block_index / blockimg_cols_ + block_rows_; ++cell_row ) {
			for( Index cell_col = block_index % blockimg_cols_; cell_col < block_index % blockimg_cols_ + block_cols_; ++cell_col ) {
				index_list.push_back( cell_row*cell_cols_ + cell_col );
			}
		}
	}

	/**
	 * @brief Get gradient of the pixel with the coordinate
	 * @param[in] image Input image
	 * @param[in] pixel_row Index of row of the pixel
	 * @param[in] pixel_col Index of column of the pixel
	 * @return Gradient
	 */
	Gradient GetGradient( const cv::Mat& image, const Index& pixel_row, const Index& pixel_col ) const {
		size_t image_width = static_cast<size_t>( image.cols );
		size_t image_height = static_cast<size_t>( image.rows );
		assert( pixel_row >= 0 && pixel_row < image_height && pixel_col >=0 && pixel_col < image_width );
		/* check image type */
		assert( image.depth() == CV_8U );
		assert( image.channels() == 1 );

		Gradient gradient;
		double x_grad, y_grad, gradient_angle;

		if( pixel_col == 0 ) {
			x_grad = cv::saturate_cast<double>(image.at<uchar>(pixel_row, 1)) - cv::saturate_cast<double>(image.at<uchar>(pixel_row, 0));
		} else if( pixel_col == image_width - 1 ) {
			x_grad = cv::saturate_cast<double>(image.at<uchar>(pixel_row, image_width - 1)) - cv::saturate_cast<double>(image.at<uchar>(pixel_row, image_width - 2));
		} else {
			x_grad = cv::saturate_cast<double>(image.at<uchar>(pixel_row, pixel_col + 1)) - cv::saturate_cast<double>(image.at<uchar>(pixel_row, pixel_col - 1));
		}

		if(pixel_row == 0) {
			y_grad = cv::saturate_cast<double>(image.at<uchar>(1, pixel_col)) - cv::saturate_cast<double>(image.at<uchar>(0, pixel_col));
		}else if(pixel_row == image_height - 1) {
			y_grad = cv::saturate_cast<double>(image.at<uchar>(image_height - 1, pixel_col)) - cv::saturate_cast<double>(image.at<uchar>(image_height - 2, pixel_col));
		}else {
			y_grad = cv::saturate_cast<double>(image.at<uchar>(pixel_row + 1, pixel_col)) - cv::saturate_cast<double>(image.at<uchar>(pixel_row - 1, pixel_col));
		}

		gradient.magnitude = sqrt( pow(x_grad, 2) + pow(y_grad, 2) );
		gradient_angle = atan2( y_grad, x_grad ) + M_PI;
		if( gradient_angle > M_PI ) gradient_angle -= M_PI;
		gradient.orientation = static_cast<int>( floor(gradient_angle / unit_angle_) );

		return gradient;
	}

	/**
	 * @brief Voting cell histograms
	 * @param[in] image
	 * @param[in] pixel_row
	 * @param[in] pixel_col
	 * @param[out] cell_histograms
	 */
	void Vote( const cv::Mat& image, const Index& pixel_row, const Index& pixel_col, CellHistograms& cell_histograms ) const {
		assert( pixel_col >= 0 && static_cast<int>(pixel_col) < image.cols && pixel_row >= 0 && static_cast<int>(pixel_row) < image.rows );
		Index cell_index = GetCellIndex(pixel_row, pixel_col);
		Gradient gradient = GetGradient( image, pixel_row, pixel_col );
		cell_histograms(cell_index, gradient.orientation) += gradient.magnitude;
	}

	/**
	 * @brief Normalizing histograms of the block with (block_row, block_col) and put them to the descriptors
	 * @param[in] cell_histograms
	 * @param[in] block_index
	 * @param[out] descriptors
	 */
	void CalcBlockDescriptors( const CellHistograms& cell_histograms, const Index& block_index, std::vector<T>& descriptors ) const {
		assert( descriptors.size() == dimension_ );

		std::vector<Index> cell_index_list;
		GetCellIndexList( block_index, cell_index_list );

		size_t block_descriptor_size = grad_orientation_num_ * block_rows_ * block_cols_;
		std::vector<double> block_descriptor;
		block_descriptor.reserve( block_descriptor_size );
		double normalize_constant = 0.0;
		for( Index cell = 0; cell < block_rows_*block_cols_; ++cell ) {
			for( Index bin = 0; bin < grad_orientation_num_; ++bin ) {
				block_descriptor.push_back( cv::saturate_cast<double>( cell_histograms( cell_index_list[cell], bin ) ) );
				normalize_constant += pow( block_descriptor.back(), 2 );
			}
		}
		normalize_constant += pow( epsilon_, 2 );
		normalize_constant = sqrt( normalize_constant );

		Index descriptor_begin_index = block_index * block_descriptor_size;
		for( Index ind = 0; ind < block_descriptor.size(); ++ind ) {
			if( normalize_constant < epsilon_ ) {
				descriptors[ descriptor_begin_index + ind ] = 0;
			} else {
				descriptors[ descriptor_begin_index + ind ] = static_cast<T>( block_descriptor[ind] / normalize_constant );
			}
		}
	}
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* HOG_EXTRACTOR_HPP_ */
