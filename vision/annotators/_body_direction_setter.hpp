/*
 * body_direction_setter.hpp
 *
 *  Created on: 2012/11/21
 *      Author: takahashi
 */

#ifndef BODY_DIRECTION_SETTER_HPP_
#define BODY_DIRECTION_SETTER_HPP_

#include "human_image_labels.hpp"
#include "tcpp/ip/visualize/show_roi.hpp"

#include <cmath>
#include <boost/array.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace human_image_labelling {

template <size_t direction_num_>
class BodyDirectionSetter {
public:
	BodyDirectionSetter( const cv::Mat& image, HumanImageLabels& labels, HumanImageLabels reference_labels = HumanImageLabels() ):
		labels_(labels), selected_label_(reference_labels.body_direction()), current_label_(labels.body_direction()),
		center_(image.size().width/2, image.size().height/2),
		radius_x_((image.size().width-10)*0.5), radius_y_(radius_x_),
		angle_unit_(360/direction_num_), division_angles_(),
		unselected_color_(255, 0, 0), selected_color_(0, 0, 255), fixed_color_(0, 255, 0)
	{
		for(unsigned int i=0 ; i<direction_num_+1 ; ++i) {
			division_angles_[i] = angle_unit_*i;
		}

		if(selected_label_ == current_label_)
			selected_label_ = -1;

		preview_base_img_ = image.clone();

		std::string label_text = "Body Direc.";
		int font_face = cv::FONT_HERSHEY_SIMPLEX;
		int thickness = 1;
		int baseline = 0;
		cv::Size text_size = cv::getTextSize(label_text, font_face, 1.0, thickness, &baseline);
		baseline += thickness;
		double font_scale = static_cast<double>(preview_base_img_.size().width - 10) / static_cast<double>(text_size.width);
		cv::putText(preview_base_img_, label_text, cv::Point(5, preview_base_img_.size().height-5), font_face, font_scale, cv::Scalar(255,0,255), thickness, CV_AA);

		SelectLabel(selected_label_);
		MakePreviewImg();
	}

	virtual ~BodyDirectionSetter() {}

	const cv::Mat& preview_img() { return preview_img_; }

	void SetLabel( unsigned int x, unsigned int y ) {
		int label = GetLabel(x, y);
		SelectLabel(label);
	}

	void SetLabel( int label ) {
		if(label < direction_num_)
			SelectLabel(label);
	}

	void Fix() {
		if( selected_label_ != -1 ) {
			current_label_ = selected_label_;
			selected_label_ = -1;
			SelectLabel(selected_label_);
			labels_.set_body_direction(current_label_);
			labels_.Save();
		}
	}

	void Reset() {
		current_label_ = -1;
		selected_label_ = -1;
		SelectLabel(selected_label_);
		labels_.set_body_direction(current_label_);
		labels_.Save();
	}

private:
	HumanImageLabels& labels_;
	cv::Mat preview_base_img_, preview_img_;
	int selected_label_, current_label_;


	cv::Point center_;
	double radius_x_, radius_y_, angle_unit_;
	boost::array<double, direction_num_+1> division_angles_;
	cv::Scalar unselected_color_, selected_color_, fixed_color_;
	boost::array<cv::Scalar, direction_num_> color_array_;

	int GetLabel( unsigned int x, unsigned int y ) {
		double xx = static_cast<double>(static_cast<int>(x) - center_.x);
		double yy = static_cast<double>(center_.y - static_cast<int>(y));
		double radian = std::atan2(yy, xx);
		if(radian < 0.0)
			radian += 2*M_PI;
		double angle = radian * 180.0 / M_PI;
		angle += 90.0 + angle_unit_*0.5;
		if(angle >= 360.0)
			angle -= 360.0;

		int label = -1;
		for(unsigned int i=0 ; i<direction_num_ ; ++i) {
			if(angle >= division_angles_[i] && angle < division_angles_[i+1]) {
				label = i;
			}
		}

		return label;
	}

	void SelectLabel( int label ) {
		selected_label_ = label;
		for(unsigned int i=0 ; i<direction_num_ ; ++i) {
			if( static_cast<int>(i) == selected_label_ ) {
				color_array_[i] = selected_color_;
			} else if( static_cast<int>(i) == current_label_ ) {
				color_array_[i] = fixed_color_;
			} else {
				color_array_[i] = unselected_color_;
			}
		}
		MakePreviewImg();
		cv::imshow(window_name, preview_img_);
	}

	void MakePreviewImg() {
		cv::Mat circle_img = preview_base_img_.clone();
		cv::Point center(circle_img.size().width/2, circle_img.size().height/2);
		for(unsigned int i=0 ; i<direction_num_ ; ++i) {
			double angle = angle_unit_*i;
			cv::ellipse(circle_img, center, cv::Size(radius_x_, radius_y_), 90 + angle_unit_/2, -(angle+5.0), -(angle + angle_unit_-5.0), color_array_[i], -1, CV_AA);
		}
		cv::addWeighted(preview_base_img_, 0.5, circle_img, 0.5, 0.0, preview_img_);
	}
};

template <size_t direction_num>
void BodyDirectionSet( int event, int x, int y, int flags, void* _body_direction_setter ) {
	BodyDirectionSetter<direction_num>* body_direction_setter = reinterpret_cast<BodyDirectionSetter<direction_num>*>(_body_direction_setter);

	switch(event) {
	case cv::EVENT_LBUTTONDOWN:
		body_direction_setter->SetLabel( x, y );
		break;
	}
}

} /* namespace human_image_labelling */

#endif /* BODY_DIRECTION_SETTER_HPP_ */
