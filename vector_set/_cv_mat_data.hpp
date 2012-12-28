/*
 * cv_mat_data.hpp
 *
 *  Created on: 2012/11/08
 *      Author: takahashi
 */

#ifndef TCPP_CV_MAT_DATA_HPP_
#define TCPP_CV_MAT_DATA_HPP_

#include "../core.hpp"
#include "../../ip/roi.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

namespace tcpp {

enum MatSort {
	kRowByRow,
	kColByCol
};

template <typename T, size_t dimension>
void Merge( const cv::Mat& mat1, const cv::Mat& mat2, cv::Mat& dst_mat ) {
	cv::Mat merged_mat;
	if( mat1.rows > 0 && mat2.rows == 0 ) {
		merged_mat = mat1.clone();
	} else if( mat1.rows == 0 && mat2.rows > 0 ) {
		merged_mat = mat2.clone();
	} else if( mat1.rows > 0 && mat2.rows > 0 ) {
		assert( mat1.type() == mat2.type() );
		int total_data_num = mat1.rows + mat2.rows;
		merged_mat = cv::Mat::zeros(total_data_num, 1, mat1.type());
		for(int i=0 ; i<total_data_num ; ++i) {
			if(i<mat1.rows) {
				merged_mat.at<cv::Vec<T, dimension> >(i, 0) = mat1.at<cv::Vec<T, dimension> >(i, 0);
			}else{
				merged_mat.at<cv::Vec<T, dimension> >(i, 0) = mat2.at<cv::Vec<T, dimension> >(i - mat1.rows, 0);
			}
		}
	}
	dst_mat = merged_mat.clone();
}

template <typename T, size_t dimension, int cv_type>
void MultiMat2Single(const cv::Mat& multi_mat, const MatSort& sort, cv::Mat& single_mat) {
	assert( multi_mat.channels() == dimension );
	if(sort == kRowByRow) {
		single_mat = cv::Mat::zeros(multi_mat.rows, dimension, cv_type);
		for(int row=0 ; row<multi_mat.rows ; ++row)
			for(size_t dim=0 ; dim<dimension ; ++dim)
				single_mat.at<T>(row, dim) = multi_mat.at<cv::Vec<T, dimension> >(row, 0)[dim];
	}else if(sort == kColByCol){
		single_mat = cv::Mat::zeros(dimension, multi_mat.rows, cv_type);
		for(int row=0 ; row<multi_mat.rows ; ++row)
			for(size_t dim=0 ; dim<dimension ; ++dim)
				single_mat.at<T>(dim, row) = multi_mat.at<cv::Vec<T, dimension> >(row, 0)[dim];
	}
}

template <typename T>
void GetROI(const cv::Mat& cv_mat, tcpp::ip::ROI<int>& roi) {
	assert( cv_mat.channels() == 2 );
	int data_num = cv_mat.rows;
	namespace boost_acc = boost::accumulators;
	boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::min, boost_acc::tag::max> > acc_x;
	boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::min, boost_acc::tag::max> > acc_y;
	for(int i=0 ; i<data_num ; ++i) {
		acc_x(static_cast<double>(cv_mat.at<cv::Vec<T, 2> >(i)[0]));
		acc_y(static_cast<double>(cv_mat.at<cv::Vec<T, 2> >(i)[1]));
	}
	int x0 = static_cast<int>(boost_acc::min(acc_x));
	int y0 = static_cast<int>(boost_acc::min(acc_y));
	int x1 = static_cast<int>(boost_acc::max(acc_x));
	int y1 = static_cast<int>(boost_acc::max(acc_y));
	roi = tcpp::ip::ROI<int>(x0, y0, x1, y1);
}

template <typename T, size_t dimension>
double HausdorffDistance(const cv::Mat& src_mat, const cv::Mat& dst_mat) {
	assert( src_mat.channels() == dst_mat.channels() );
	int src_data_num = src_mat.rows;
	int dst_data_num = dst_mat.rows;

	namespace boost_acc = boost::accumulators;
	boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::max> > acc1;
	for(int i=0 ; i<src_data_num ; ++i) {
		boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::min> > acc2;
		for(int j=0 ; j<dst_data_num ; ++j) {
			double distance = cv::norm( dst_mat.at<cv::Vec<T, dimension> >(j, 0) - src_mat.at<cv::Vec<T, dimension> >(i, 0) );
			acc2(distance);
		}
		acc1(boost_acc::min(acc2));
	}
	double max_min_distance = boost_acc::max(acc1);
	return max_min_distance;
}

template <typename T, size_t dimension>
double Distance(const cv::Mat& src_mat, const cv::Mat& dst_mat) {
	assert( src_mat.channels() == dst_mat.channels() );

	int src_data_num = src_mat.rows;
	int dst_data_num = dst_mat.rows;

	namespace boost_acc = boost::accumulators;
	boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::min> > acc;
	for(int i=0 ; i<src_data_num ; ++i) {
		for(int j=0 ; j<dst_data_num ; ++j) {
			double distance = cv::norm( dst_mat.at<cv::Vec<T, dimension> >(j, 0) - src_mat.at<cv::Vec<T, dimension> >(i, 0) );
			acc(distance);
		}
	}
	double min_distance = boost_acc::min(acc);
	return min_distance;
}

template <typename T, size_t dimension>
double Distance(const cv::Vec<T, dimension>& vec, const cv::Mat& mat) {
	assert( mat.channels() == dimension );

	namespace boost_acc = boost::accumulators;
	boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::min> > acc;
	for(int i=0 ; i<mat.rows ; ++i) {
		double distance = cv::norm( mat.at<cv::Vec<T, dimension> >(i, 0) - vec );
		acc(distance);
	}
	double min_distance = boost_acc::min(acc);
	return min_distance;
}

template <typename T, size_t dimension, int cv_type>
double MahalanobisDistance(const cv::Vec<T, dimension>& vec, const cv::Mat& mat) {
	double distance;
	if(mat.rows < 2) {
		distance = cv::norm(vec - mat.at<cv::Vec<T, dimension> >(0));
	} else {
		cv::Mat vec_mat, single_mat;
		tcpp::Convert<T, dimension, cv_type>(vec, vec_mat);
		MultiMat2Single<T, dimension, cv_type>(mat, kRowByRow, single_mat);
		cv::Mat covariance, mean, icovariance;
		cv::calcCovarMatrix(single_mat, covariance, mean, CV_COVAR_NORMAL|CV_COVAR_ROWS, cv_type);
		cv::invert(covariance, icovariance, cv::DECOMP_SVD);
		if(vec_mat.size() != mean.size())
			distance = cv::Mahalanobis(vec_mat, mean.t(), icovariance);
	}
	return distance;
}

} /* namespace tcpp */

#endif /* TCPP_CV_MAT_DATA_HPP_ */
