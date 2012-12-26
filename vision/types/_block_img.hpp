/*
 * block_img.hpp
 *
 *  Created on: 2012/11/29
 *      Author: takahashi
 */

#ifndef TCPP_BLOCK_IMG_HPP_
#define TCPP_BLOCK_IMG_HPP_

#include "../core/core.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace tcpp {
namespace ip {

template <typename T>
void GetMeanBlocks(const cv::Mat &image, const cv::Size2i &block_size, Eigen::Array<T, -1, -1> &mean_array) {
	assert(image.channels() == 1 || image.depth() == CV_8U);
	if(image.cols%block_size.width != 0 || image.rows%block_size.height != 0) {
		std::cerr << WARNING_HEAD_STREAM << "Block size may be inappropriate." << std::endl;
	}
	int block_cols = image.cols/block_size.width, block_rows = image.rows/block_size.height;
	mean_array = Eigen::ArrayXXd::Zero(block_rows, block_cols);
	for(int block_row=0 ; block_row<block_rows ; ++block_row) {
		for(int block_col=0 ; block_col<block_cols ; ++block_col) {
			int row_begin = block_size.height * block_row, row_end = row_begin + block_size.height-1;
			int col_begin = block_size.width * block_col, col_end = col_begin + block_size.width-1;
			namespace boost_acc = boost::accumulators;
			boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::mean> > acc;
			for(int row=row_begin ; row<row_end ; ++row)
				for(int col=col_begin ; col<col_end ; ++col)
					acc(cv::saturate_cast<int>(image.at<uchar>(row, col)));
			mean_array.coeffRef(block_row, block_col) = static_cast<T>( boost_acc::mean(acc) / 256.0 );
		}
	}
}

template <typename T, size_t array_rows, size_t array_cols>
void Array2Mat(const Eigen::Array<T, array_rows, array_cols>& array, const cv::Size2i& block_size, cv::Mat& image) {
	cv::resize(image, image, cv::Size(array.cols()*block_size.width, array.rows()*block_size.height));
	for(int block_row=0 ; block_row<array.rows() ; ++block_row) {
		for(int block_col=0 ; block_col<array.cols() ; ++block_col) {
			int row_begin = block_size.height * block_row, row_end = row_begin + block_size.height;
			int col_begin = block_size.width * block_col, col_end = col_begin + block_size.width;
			image(cv::Range(row_begin, row_end), cv::Range(col_begin, col_end)) = cv::Mat::ones(block_size, CV_32F)*static_cast<double>(array.coeff(block_row, block_col));
		}
	}
}

} /* namespace ip */
} /* namespace tcpp */


#endif /* TCPP_BLOCK_IMG_HPP_ */
