/**
 * @file human_image_labels.hpp
 * @brief Label set class of human image
 * @author Toshimitsu Takahashi
 * @date 2013/1/10
 * @version 0.0.1
 *
 */

#ifndef TCPP_HUMAN_IMAGE_LABELS_HPP_
#define TCPP_HUMAN_IMAGE_LABELS_HPP_

#include "tcpp2/vision/types/roi.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

namespace boost_fs = boost::filesystem;

/**
 * namespace tcpp
 */
namespace tcpp {
/**
 * namespace vision
 */
namespace vision {

enum LabelType {
	kHeadROI = 0,
	kHeadDirection = 1,
	kBodyDirection = 2
};

class HumanImageLabels {
public:
	HumanImageLabels(): image_name_(), label_num_(3), head_roi_(), head_direction_(-1), body_direction_(-1) {}

	const std::string& image_name() { return image_name_; }
	const tcpp::vision::ROI<int>& head_roi() const { return head_roi_; }
	const int& head_direction() const { return head_direction_; }
	const int& body_direction() const { return body_direction_; }
	int label_num() const { return label_num_; }

	void set_head_roi( const tcpp::vision::ROI<int>& head_roi ) { head_roi_ = head_roi; }
	void set_head_direction( const int& head_direction ) { head_direction_ = head_direction; }
	void set_body_direction( const int& body_direction ) { body_direction_ = body_direction; }

	int Save( const std::string& xml_path ) {
		std::ofstream ofs( xml_path.c_str() );
		if( ofs ) {
			boost::archive::xml_oarchive oarchive(ofs);
			oarchive << boost::serialization::make_nvp("HumanImageLabels", *this);
			ofs.close();
			return 0;
		} else {
			return 1;
		}
	}

	int Load( const std::string& xml_path ) {
		std::ifstream ifs( xml_path.c_str() );
		if( ifs ) {
			boost::archive::xml_iarchive iarchive( ifs );
			iarchive >> boost::serialization::make_nvp( "HumanImageLabels", *this );
			ifs.close();
			return 0;
		} else {
			return 1;
		}
	}

private:
	std::string image_name_;
	int label_num_;
	tcpp::vision::ROI<int> head_roi_;
	int head_direction_, body_direction_;

	friend class boost::serialization::access;
	template <class Archive>
	void serialize( Archive& archive, unsigned int version ) {
		if(version == 2) {
			archive & boost::serialization::make_nvp("label_num", label_num_);
			label_num_ = 3;
		}
		archive & boost::serialization::make_nvp("head_roi", head_roi_);
		archive & boost::serialization::make_nvp("head_direction", head_direction_);
		if(version >= 3) {
			archive & boost::serialization::make_nvp("body_direction", body_direction_);
		}
	}
};

} /* vision */
} /* namespace tcpp */

BOOST_CLASS_VERSION(tcpp::vision::HumanImageLabels, 3)

#endif /* TCPP_HUMAN_IMAGE_LABELS_HPP_ */
