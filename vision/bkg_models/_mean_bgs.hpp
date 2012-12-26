/*
 * mean_bgs.hpp
 *
 *  Created on: 2012/11/28
 *      Author: takahashi
 */

#ifndef TCPP_MEAN_BGS_HPP_
#define TCPP_MEAN_BGS_HPP_

#include "../../core/core.hpp"

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

namespace tcpp {
namespace ip {

class MeanBGS {
public:
	MeanBGS(): count_(0) {}

	virtual ~MeanBGS() {}

	void operator<<( const cv::Mat& bkg_img ) {
		if( count_ >= 1 ) {
			assert( bkg_img.size() == mean_mat_.size() && bkg_img.channels() == mean_mat_.channels() );
			double alpha = 1.0 / ( 1.0 + static_cast<double>(count_) );
			cv::Mat _bkg_img;
			bkg_img.convertTo(_bkg_img, CV_64F);
			mean_mat_ = (1.0 - alpha) * mean_mat_ + alpha * _bkg_img;
			variance_mat_ = (1.0 - alpha) * variance_mat_ + alpha * (1.0 - alpha) * (_bkg_img - mean_mat_).mul(_bkg_img - mean_mat_);
		} else {
			bkg_img.convertTo(mean_mat_, CV_64F);
			variance_mat_ = cv::Mat::zeros( mean_mat_.size(), mean_mat_.type() );
		}
		++count_;
	}

	void GenerateModel( double low_threshold = 2.0, double high_threshold = 2.0 ) {
		cv::Mat std_dev_mat;
		cv::sqrt( variance_mat_, std_dev_mat );
		low_mat_ = mean_mat_ - std_dev_mat * low_threshold;
		high_mat_ = mean_mat_ + std_dev_mat * high_threshold;
		count_ = 0;
	}

	void Subtract( const cv::Mat& img, cv::Mat& mask ) {
		assert( img.size() == low_mat_.size() && img.channels() == low_mat_.channels() );
		cv::Mat _low_mat, _high_mat;
		low_mat_.convertTo(_low_mat, img.type());
		high_mat_.convertTo(_high_mat, img.type());
		cv::inRange( img, _low_mat, _high_mat, mask );
		mask = 255 - mask;
	}

	void Subtract( const cv::Mat& img, const cv::Point2i& left_upper, cv::Mat& mask ) {
		assert( img.channels() == low_mat_.channels() );
		cv::Point2i right_lower(left_upper.x+img.cols -1, left_upper.y+img.rows -1);
		cv::Rect_<int> img_rect(0, 0, low_mat_.cols, low_mat_.rows);
		assert( img_rect.contains( left_upper ) && img_rect.contains( right_lower ) );
		cv::Rect_<int> roi(left_upper.x, left_upper.y, img.cols, img.rows);
		cv::Mat _low_mat, _high_mat;
		low_mat_.convertTo(_low_mat, img.type());
		high_mat_.convertTo(_high_mat, img.type());
		cv::Mat low_submat(_low_mat, roi), high_submat(_high_mat, roi);
		cv::inRange( img, low_submat, high_submat, mask );
		mask = 255 - mask;
	}

	void Save( const boost::filesystem::path& xml_path ) {
		if( !boost::filesystem::exists(xml_path.parent_path()) ) {
			PRINT_WARNING( xml_path.parent_path().string() + " doesn't exist." );
			boost::filesystem::create_directories( xml_path.parent_path() );
			std::cout << "Created " << xml_path.parent_path() << std::endl;
		}
		cv::FileStorage storage( xml_path.c_str(), CV_STORAGE_WRITE );
		cv::write( storage, "low_mat", low_mat_ );
		cv::write( storage, "high_mat", high_mat_ );
	}

	void Load( const boost::filesystem::path& xml_path ){
		if( !boost::filesystem::exists(xml_path) ) {
			PRINT_ERROR( xml_path.string() + " doesn't exist." );
			abort();
		}
		cv::FileStorage storage( xml_path.c_str(), CV_STORAGE_READ );
		cv::FileNode node( storage.fs, NULL );
		cv::read( node["low_mat"], low_mat_ );
		cv::read( node["high_mat"], high_mat_ );
		count_ = 0;
	}

private:
	cv::Mat mean_mat_, variance_mat_;
	cv::Mat low_mat_, high_mat_;
	int count_;
};

} /* namespace ip */
} /* namespace tcpp */

#endif /* TCPP_MEAN_BGS_HPP_ */
