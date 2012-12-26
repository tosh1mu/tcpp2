/*
 * human_image_labels.hpp
 *
 *  Created on: 2012/11/20
 *      Author: takahashi
 */

#ifndef HUMAN_IMAGE_LABELS_HPP_
#define HUMAN_IMAGE_LABELS_HPP_

#include "tcpp/core/core.hpp"
#include "tcpp/ip/roi.hpp"

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

namespace human_image_labelling {

enum LabelType {
	kHeadROI = 0,
	kHeadDirection = 1,
	kBodyDirection = 2
};

const std::string window_name = "Human Image Labelling";

class HumanImageLabels {
public:
	HumanImageLabels(): xml_path_(), image_name_(), label_num_(3), head_roi_(), head_direction_(-1), body_direction_(-1) {}

	HumanImageLabels( const boost_fs::path& xml_path, const int save = 1 ): xml_path_(xml_path), image_name_(xml_path.stem().string()), label_num_(3),
	head_roi_(), head_direction_(-1), body_direction_(-1)
	{
		if( !Load() && save ) Save();
	}

	HumanImageLabels( const boost_fs::path& xml_path, const HumanImageLabels& labels, const int save = 1 ): xml_path_(xml_path), image_name_(xml_path.stem().string()), label_num_(3),
			head_roi_(labels.head_roi_), head_direction_(labels.head_direction_), body_direction_(labels.body_direction_)
	{
		if( !Load() && save ) Save();
	}

	virtual ~HumanImageLabels() {}

	const std::string& image_name() { return image_name_; }
	const tcpp::ip::ROI<int>& head_roi() const { return head_roi_; }
	const int& head_direction() const { return head_direction_; }
	const int& body_direction() const { return body_direction_; }
	int label_num() const { return label_num_; }

	void set_head_roi( const tcpp::ip::ROI<int>& head_roi ) { head_roi_ = head_roi; }
	void set_head_direction( const int& head_direction ) { head_direction_ = head_direction; }
	void set_body_direction( const int& body_direction ) { body_direction_ = body_direction; }

	void Save() {
		std::ofstream ofs( xml_path_.c_str() );
		boost::archive::xml_oarchive oarchive(ofs);
		oarchive << boost::serialization::make_nvp("HumanImageLabels", *this);
		ofs.close();
	}

	int Load() {
		if( boost::filesystem::exists( xml_path_ ) ) {
			std::ifstream ifs( xml_path_.c_str() );
			boost::archive::xml_iarchive iarchive( ifs );
			iarchive >> boost::serialization::make_nvp( "HumanImageLabels", *this );
			ifs.close();
			return 1;
		}else{
			return 0;
		}
	}

private:
	boost_fs::path xml_path_;
	std::string image_name_;
	int label_num_;
	tcpp::ip::ROI<int> head_roi_;
	int head_direction_, body_direction_;

	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive& archive, unsigned int version) {
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

} /* namespace human_image_labelling */

BOOST_CLASS_VERSION(human_image_labelling::HumanImageLabels, 3)

#endif /* HUMAN_IMAGE_LABELS_HPP_ */
