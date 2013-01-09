/**
 * @file visualize_roi.hpp
 * @brief Functions to visualize tcpp::vision::ROI<T>
 * @author Toshimitsu Takhashi
 * @date 2013/1/10
 * @version 0.0.1
 *
 */

#ifndef TCPP_VISUALIZE_ROI_HPP_
#define TCPP_VISUALIZE_ROI_HPP_

#include "tcpp2/vision/types/roi.hpp"
#include "tcpp2/core/convert/string_numeric_convert.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

template <typename T>
void DrawROI(const cv::Mat& src_img, const ROI<T>& roi, cv::Mat& dst_img, cv::Scalar color = cv::Scalar(0, 0, 255), int thickness = 2, int line_type = 8 ) {
	dst_img = src_img.clone();
	if( dst_img.channels() == 1 )
		cv::cvtColor( dst_img, dst_img, CV_GRAY2BGR );
	cv::Point2i left_upper = roi.left_upper(), right_lower = roi.right_lower();
	cv::rectangle(dst_img, left_upper, right_lower, color, thickness, line_type);
}

template <typename T>
void DrawFilledROI(const cv::Mat& src_img, const ROI<T>& roi, cv::Mat& dst_img, cv::Scalar color = cv::Scalar(0, 0, 255), double roi_rate = 0.4, double src_rate = 0.6, double add = 0.0) {
	cv::Mat roi_image = src_img.clone();
	cv::Mat _src_img = src_img.clone();
	if( roi_image.channels() == 1 ) {
		cv::cvtColor( roi_image, roi_image, CV_GRAY2BGR );
		cv::cvtColor( _src_img, _src_img, CV_GRAY2BGR );
	}
	DrawROI(roi_image, roi, roi_image, color, -1);
	cv::addWeighted(_src_img, src_rate, roi_image, roi_rate, add, dst_img);
}

template<typename T>
void DrawROIs(const cv::Mat &src_img, const std::vector<ROI<T> > &rois, cv::Mat &dst_img, cv::Scalar color = cv::Scalar(0, 0, 255)) {
	dst_img = src_img.clone();
	if( dst_img.channels() == 1 )
		cv::cvtColor( dst_img, dst_img, CV_GRAY2BGR );
	for( unsigned int i=0 ; i<rois.size() ; ++i ) {
		cv::Point2i left_upper = rois[i].left_upper(), right_lower = rois[i].right_lower();
		cv::rectangle( dst_img, left_upper, right_lower, color, 2, 3 );
	}
}

template<typename T>
void DrawROIs(const cv::Mat &src_img, const std::map<int, ROI<T> > &rois, cv::Mat &dst_img, cv::Scalar rect_color = cv::Scalar(0, 0, 255), cv::Scalar label_color = cv::Scalar(0,200,0)) {
	dst_img = src_img.clone();
	if( dst_img.channels() == 1 )
		cv::cvtColor( dst_img, dst_img, CV_GRAY2BGR );
	typename std::map<int, ROI<T> >::const_iterator roi_itr = rois.begin();
	while(roi_itr != rois.end()) {
		int label = (*roi_itr).first;
		ROI<T> roi = (*roi_itr).second;
		cv::Point_<T> left_upper = roi.left_upper(), right_lower = roi.right_lower(), right_upper = roi.right_upper();
		cv::rectangle( dst_img, left_upper, right_lower, rect_color, 2, 3);
		cv::putText(dst_img, tcpp::string(label), right_upper, cv::FONT_HERSHEY_SIMPLEX,
					0.7, label_color, 2, CV_AA);
		++roi_itr;
	}
}

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_VISUALIZE_ROI_HPP_ */
