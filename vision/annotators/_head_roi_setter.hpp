/*
 * head_roi_setter.hpp
 *
 *  Created on: 2012/11/20
 *      Author: takahashi
 */

#ifndef HEAD_ROI_SETTER_HPP_
#define HEAD_ROI_SETTER_HPP_

#include "human_image_labels.hpp"
#include "tcpp/ip/visualize/show_roi.hpp"

#include <opencv2/highgui/highgui.hpp>

namespace human_image_labelling {

class HeadROISetter {
public:
	HeadROISetter( const cv::Mat& image, HumanImageLabels& labels, HumanImageLabels reference_labels = HumanImageLabels() ):
		labels_(labels), fixed_roi_(labels.head_roi()), temporary_roi_(reference_labels.head_roi()),
		fixed_color_(0, 200, 0), temporary_color_(0, 0, 255), setting_color_(0, 0, 255)
	{
		preview_base_img_ = image.clone();

		std::string label_text = "Head ROI";
		int font_face = cv::FONT_HERSHEY_SIMPLEX;
		int thickness = 1;
		int baseline = 0;
		cv::Size text_size = cv::getTextSize(label_text, font_face, 1.0, thickness, &baseline);
		baseline += thickness;
		double font_scale = static_cast<double>(preview_base_img_.size().width - 10) / static_cast<double>(text_size.width);
		cv::putText(preview_base_img_, label_text, cv::Point(5, preview_base_img_.size().height-5), font_face, font_scale, cv::Scalar(255,0,255), thickness, CV_AA);

		tcpp::ip::DrawFilledROI<int>(preview_base_img_, fixed_roi_, preview_img_, fixed_color_);
	}

	virtual ~HeadROISetter() {}

	const cv::Mat& preview_img() { return preview_img_; }

	void SetLeftUpper( unsigned int x, unsigned int y ) {
		temporary_roi_.set_x0(x);
		temporary_roi_.set_y0(y);
		cv::imshow(window_name, preview_img_);
	}

	void ShowSettingROI( unsigned int x, unsigned int y ) {
		cv::Mat setting_img;
		tcpp::ip::ROI<int> setting_roi(temporary_roi_.x0(), temporary_roi_.y0(), x, y);
		tcpp::ip::DrawROI(preview_img_, setting_roi, setting_img, setting_color_, 1);
		cv::imshow(window_name, setting_img);
	}

	void ShowTemporaryROI() {
		cv::Mat temp_img = preview_img_.clone();
		tcpp::ip::DrawROI(temp_img, temporary_roi_, temp_img, temporary_color_, 1);
		cv::imshow(window_name, temp_img);
	}

	void SetRightLower( unsigned int x, unsigned int y ) {
		temporary_roi_.set_x1(x);
		temporary_roi_.set_y1(y);
		ShowTemporaryROI();
	}

	void MoveLeft( unsigned int px = 1 ) {
		temporary_roi_.MoveLeft(px);
		temporary_roi_.Adjust(preview_img_.size());
		ShowTemporaryROI();
	}

	void MoveRight( unsigned int px = 1 ) {
		temporary_roi_.MoveRight(px);
		temporary_roi_.Adjust(preview_img_.size());
		ShowTemporaryROI();
	}

	void MoveUp( unsigned int px = 1 ) {
		temporary_roi_.MoveUp(px);
		temporary_roi_.Adjust(preview_img_.size());
		ShowTemporaryROI();
	}

	void MoveDown( unsigned int px = 1 ) {
		temporary_roi_.MoveDown(px);
		temporary_roi_.Adjust(preview_img_.size());
		ShowTemporaryROI();
	}

	void ExpandWidth( unsigned int px = 1 ) {
		temporary_roi_.ExpandHorizontal(px);
		temporary_roi_.Adjust(preview_img_.size());
		ShowTemporaryROI();
	}

	void Fix() {
		fixed_roi_ = temporary_roi_;
		temporary_roi_ = tcpp::ip::ROI<int>();
		tcpp::ip::DrawFilledROI<int>(preview_base_img_, fixed_roi_, preview_img_, fixed_color_);
		cv::imshow(window_name, preview_img_);
		labels_.set_head_roi(fixed_roi_);
		labels_.Save();
	}

	void Reset() {
		fixed_roi_ = tcpp::ip::ROI<int>();
		temporary_roi_ = tcpp::ip::ROI<int>();
		preview_img_ = preview_base_img_.clone();
		cv::imshow(window_name, preview_img_);
		labels_.set_head_roi(fixed_roi_);
		labels_.Save();
	}

private:
	HumanImageLabels& labels_;
	cv::Mat preview_base_img_, preview_img_;
	tcpp::ip::ROI<int> fixed_roi_, temporary_roi_;
	cv::Scalar fixed_color_, temporary_color_, setting_color_;
};

void HeadROISet( int event, int x, int y, int flags, void* _head_roi_setter ) {
	HeadROISetter* head_roi_setter = reinterpret_cast<HeadROISetter*>(_head_roi_setter);

	switch(event) {
	case cv::EVENT_MOUSEMOVE:
		if(flags & cv::EVENT_FLAG_LBUTTON) {
			head_roi_setter->ShowSettingROI(x, y);
		}else if(flags & cv::EVENT_FLAG_RBUTTON){
			head_roi_setter->ShowSettingROI(x, y);
		}
		break;
	case cv::EVENT_LBUTTONDOWN:
		head_roi_setter->SetLeftUpper(x, y);
		break;
	case cv::EVENT_LBUTTONUP:
		head_roi_setter->SetRightLower(x, y);
		break;
	case cv::EVENT_RBUTTONDOWN:
		head_roi_setter->SetRightLower(x, y);
		break;
	case cv::EVENT_RBUTTONUP:
		head_roi_setter->SetRightLower(x, y);
		break;
	}
}

} /* namespace human_image_labelling */

#endif /* HEAD_ROI_SETTER_HPP_ */
