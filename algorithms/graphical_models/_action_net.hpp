/*
 * action_net.h
 *
 *  Created on: 2012/10/11
 *      Author: takahashi
 */

#ifndef TCPP_ACTION_NET_HPP_
#define TCPP_ACTION_NET_HPP_

#define DISPLAY_DETAIL
#define USING_BOOST_SERIALIZATION
//
#include "../../core/core.hpp"
#include "../../core/convert.hpp"
#include "../../core/util/file.hpp"
#include "../../ip/features2d/shape_context.hpp"
//	Standard C++ Library
#include <string>
#include <vector>
//	boost C++ Library
#include <boost/algorithm/string/classification.hpp> // is_any_of
#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#ifdef USING_BOOST_SERIALIZATION
#include <fstream>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#endif
//	Eigen C++ Library
#include <eigen3/Eigen/Core>
//	Opencv Library
#include <opencv2/highgui/highgui.hpp>
// TODO Fix all
namespace tcpp {
namespace prml {

enum Action {
	kStanding,
	kPickupStanding
};

static std::string GetActionName(const Action &action) {
	switch(action) {
	case kPickupStanding:
		return "pickup-stand";
	default:
		return "standing";
	}
}

enum PoserFigure {
	kMannequin,
};

static std::string GetPoserFigureName(const PoserFigure &figure) {
	switch(figure) {
	case kMannequin:
		return "mannequin";
	default:
		return "mannequin";
	}
}

class ActionNet {
public:
	ActionNet(): model_dir_(), sc_point_num_(0), sc_angle_bin_num_(0), sc_radius_bin_num_(0) {};
	ActionNet(const boost::filesystem::path model_dir, const Action &action, const PoserFigure &figure, const int &sc_point_num, const int &sc_angle_bin_num, const int &sc_radius_bin_num):
		model_dir_(model_dir),
		action_name_(GetActionName(action)), figure_name_(GetPoserFigureName(figure)),
		sc_point_num_(sc_point_num), sc_angle_bin_num_(sc_angle_bin_num), sc_radius_bin_num_(sc_radius_bin_num) {
#ifdef DISPLAY_DETAIL
		std::cout << "ActionNet constructed: " << std::endl;
		PRINT_VAL2(model_dir_,"Dir"); PRINT_VAL2(action_name_,"Action"); PRINT_VAL2(figure_name_,"Figure");
		std::cout << std::endl;
		PRINT_VAL2(sc_point_num_,"SC_Point"); PRINT_VAL2(sc_angle_bin_num_,"SC_Angle"); PRINT_VAL2(sc_radius_bin_num_,"SC_Radius");
		std::cout << std::endl;
#endif
		boost::filesystem::path action_dir(action_name_), figure_dir(figure_name_), silhouette_dir("sil"), serialize_xml_name("action_net.xml");
		image_dir_ = model_dir_ / action_dir / figure_dir / silhouette_dir;
		serialize_xml_ = image_dir_ / serialize_xml_name;
	}
	virtual ~ActionNet() {};

	void Init() {
#ifdef DISPLAY_DETAIL
		std::cout << "Initializing ActionNet..." << std::endl;
#endif
		int node_num = 0;
		BOOST_FOREACH(const boost::filesystem::path& path, std::make_pair(boost::filesystem::directory_iterator(image_dir_), boost::filesystem::directory_iterator())) {
			if(!boost::filesystem::is_directory(path) && path.extension().string() == ".bmp")
				++node_num;
		}
#ifdef DISPLAY_DETAIL
		PRINT_VAL(node_num);
		std::cout << std::endl;
#endif
		if(!nodes_.empty())
			nodes_.clear();
		nodes_.reserve(node_num);
		BOOST_FOREACH(const boost::filesystem::path& path, std::make_pair(boost::filesystem::directory_iterator(image_dir_), boost::filesystem::directory_iterator())) {
			if(!boost::filesystem::is_directory(path) && path.extension().string() == ".bmp") {
				ModelFile model_file(path);
				int keypose = atoi(model_file.GetValue("keypose").c_str()), pitch = atoi(model_file.GetValue("pitch").c_str()), yaw = atoi(model_file.GetValue("yaw").c_str());
				Node model_node(keypose, pitch, yaw);
				cv::Mat model_image = cv::imread(path.string(), 0);
				model_node.Init(model_image, *this);
				nodes_.push_back(model_node);
			}
		}
#ifdef DISPLAY_DETAIL
		std::cout << "Finished node initializetion." << std::endl;
#endif
		edges_ = Eigen::ArrayXXi::Zero(node_num, node_num);
		for(int i=0 ; i<node_num ; ++i)
			for(int j=0 ; j<node_num ; ++j)
				edges_(i, j) = IsConnected(nodes_[i], nodes_[j]);
	}

	double FindOptimalPath(const std::vector<tcpp::ip::ShapeContext> &shape_context_sequence, std::vector<int> &path) {
		int length = static_cast<int>(shape_context_sequence.size());
		int node_number = static_cast<int>(nodes_.size());
		std::vector<std::vector<int> > paths(node_number);
		Eigen::ArrayXd costs = Eigen::ArrayXd::Zero(node_number);
		const double inf = 1E99;
		for(int i=0 ; i<length ; ++i) {
			for(int n=0 ; n<node_number ; ++n) {
				double cost = tcpp::ip::Match( shape_context_sequence[i], nodes_[n].shape_context() );
				Eigen::ArrayXd cost_buffer = costs;
				for(int nn=0 ; nn<node_number ; ++nn) {
					if(edges_(n, nn) == 0)
						cost_buffer(nn) += inf;
					cost_buffer(nn) += cost;
				}
				Eigen::ArrayXd::Index min_row, min_col;
				double min_cost = cost_buffer.minCoeff(&min_row, &min_col);
				costs[n] = min_cost;
				paths[n].push_back(min_row);
			}
		}
		Eigen::ArrayXd::Index min_row, min_col;
		double min_cost = costs.minCoeff(&min_row, &min_col);
		path = paths[min_row];
		return min_cost;
	}

	void GetNodeParameters(const std::vector<int>& node_path, std::vector<std::map<std::string, int> >& parameters_vector) {
		parameters_vector.reserve(node_path.size());
		for(unsigned int i=0 ; i<node_path.size() ; ++i ) {
			const Node& node = nodes_[node_path[i]];
			std::map<std::string, int> parameters;
			parameters["keypose"] = node.keypose();
			parameters["pitch"] = node.pitch();
			parameters["yaw"] = node.yaw();
			parameters_vector.push_back(parameters);
		}
	}

#ifdef USING_BOOST_SERIALIZATION
	void SaveXml() {
		std::ofstream ofs(serialize_xml_.c_str());
		boost::archive::xml_oarchive oarchive(ofs);
		oarchive << boost::serialization::make_nvp("Root", *this);
		ofs.close();
	}
	void LoadXml() {
		std::ifstream ifs(serialize_xml_.c_str());
		boost::archive::xml_iarchive iarchive(ifs);
		iarchive >> boost::serialization::make_nvp("Root", *this);
		ifs.close();
	}
#endif

private:
	class Node {
	public:
		Node(): keypose_(), pitch_(), yaw_() {};
		Node(const int &keypose, const int &pitch, const int &yaw):
			keypose_(keypose), pitch_(pitch), yaw_(yaw) {
#ifdef DISPLAY_DETAIL
			std::cout << "Node constructed: ";
			PRINT_VAL(keypose_); PRINT_VAL(pitch_); PRINT_VAL(yaw_);
			std::cout << std::endl;
#endif
		}
		virtual ~Node() {}
		int keypose() const {return keypose_;}
		int pitch() const {return pitch_;}
		int yaw() const {return yaw_;}
		const tcpp::ip::ShapeContext& shape_context() const{ return shape_context_; }

		void Init(const cv::Mat &model_image, const ActionNet &action_net) {
#ifdef DISPLAY_DETAIL
			std::cout << "Initializing Node..." << std::flush;
#endif
			shape_context_ = tcpp::ip::ShapeContext(action_net.sc_point_num_, action_net.sc_angle_bin_num_, action_net.sc_radius_bin_num_);
			shape_context_.Extract(model_image);
#ifdef DISPLAY_DETAIL
			std::cout << "Done." << std::endl;
#endif
		}

	private:
		int keypose_, pitch_, yaw_;
		tcpp::ip::ShapeContext shape_context_;
#ifdef USING_BOOST_SERIALIZATION
		friend class boost::serialization::access;
		template <class Archive>
		void serialize(Archive& archive, unsigned int version) {
			archive & boost::serialization::make_nvp("Keypose", keypose_);
			archive & boost::serialization::make_nvp("Pitch", pitch_);
			archive & boost::serialization::make_nvp("Yaw", yaw_);
			archive & boost::serialization::make_nvp("ShapeContext", shape_context_);
		}
#endif
	};
	int IsConnected(const Node &node1, const Node &node2) {
		int keypose_connection;
		if(node1.keypose() == node2.keypose() || node1.keypose() == node2.keypose()-1)
			keypose_connection = 1;
		else
			keypose_connection = 0;
	//
		int pitch_connection;
		if(node1.pitch() == node2.pitch())
			pitch_connection = 1;
		else
			pitch_connection = 0;
	//
		int yaw_connection;
		if(node1.yaw() >= node2.yaw()-10 && node1.yaw() <= node2.yaw()+10)
			yaw_connection = 1;
		else
			yaw_connection = 0;
		if(keypose_connection && pitch_connection && yaw_connection)
			return 1;
		else
			return 0;
	}
//
	class ModelFile : public tcpp::File {
	public:
		ModelFile(const boost::filesystem::path &file_path):
			File(file_path), value_number_(5), split_string_("SPLIT"), prefix_("PREFIX"), suffix_("SUFFIX"), value_order_(5, "VALUE")
		{
#ifdef USING_BOOST_SERIALIZATION
			boost::filesystem::path xml_name("action_net_model_file.xml");
			format_xml_path_ = file_path.parent_path() / xml_name;
			boost::system::error_code error;
			const bool result = boost::filesystem::exists(format_xml_path_, error);
			if( !result || error ) {
				std::cerr << "Error@ModelFile::ModelFile() : Could not find xml file in " << file_path.parent_path().string() << "." << std::endl;
				SaveXml();
				std::cerr << "Created template xml file (" << format_xml_path_.relative_path() << "). Please edit it correctly." << std::endl;
				abort();
			}else{
				LoadXml();
			}
			boost::algorithm::split(values_, name_, boost::is_any_of(split_string_.c_str()));
			assert( static_cast<int>( values_.size() - 1 ) == value_number_ );
			for(unsigned int i=1 ; i<values_.size()-1 ; i++)
				value_map_[value_order_[i]] = values_[i];
#endif
		}
		virtual ~ModelFile() {}
		std::string GetValue(const std::string &value_type) {return value_map_[value_type];}
	private:
		boost::filesystem::path format_xml_path_;
		int value_number_;
		std::string split_string_, prefix_, suffix_;
		std::vector<std::string> value_order_, values_;
		std::map<std::string, std::string> value_map_;
#ifdef USING_BOOST_SERIALIZATION
		friend class boost::serialization::access;
		template <class Archive>
		void serialize(Archive& archive, unsigned int version) {
			archive & boost::serialization::make_nvp("Value_Number", value_number_);
			archive & boost::serialization::make_nvp("Split_String", split_string_);
			archive & boost::serialization::make_nvp("Prefix", prefix_);
			archive & boost::serialization::make_nvp("Suffix", suffix_);
			archive & boost::serialization::make_nvp("Value_Order", value_order_);
		}
		void SaveXml() {
			std::ofstream ofs(format_xml_path_.c_str());
			boost::archive::xml_oarchive oarchive(ofs);
			oarchive << boost::serialization::make_nvp("Root", *this);
			ofs.close();
		}
		void LoadXml() {
			std::ifstream ifs(format_xml_path_.c_str());
			boost::archive::xml_iarchive iarchive(ifs);
			iarchive >> boost::serialization::make_nvp("Root", *this);
			ifs.close();
		}
#endif
	};
//
	boost::filesystem::path model_dir_, image_dir_, serialize_xml_;
	std::string action_name_, figure_name_;
	int sc_point_num_, sc_angle_bin_num_, sc_radius_bin_num_;
	std::vector<Node> nodes_;
	Eigen::ArrayXXi edges_;
#ifdef USING_BOOST_SERIALIZATION
	friend class boost::serialization::access;
	BOOST_SERIALIZATION_SPLIT_MEMBER();
	template <class Archive>
	void save(Archive& archive, const unsigned int version) const {
		std::vector<std::vector<int> > edge_vector;
		tcpp::Convert<int, Eigen::Dynamic, Eigen::Dynamic>(edges_, edge_vector);
		archive & boost::serialization::make_nvp("Action", action_name_);
		archive & boost::serialization::make_nvp("Figure", figure_name_);
		archive & boost::serialization::make_nvp("ShapeContext_Point_Num", sc_point_num_);
		archive & boost::serialization::make_nvp("ShapeContext_Angle_Bin_Num", sc_angle_bin_num_);
		archive & boost::serialization::make_nvp("ShapeContext_Radius_Bin_Num", sc_radius_bin_num_);
		archive & boost::serialization::make_nvp("Nodes", nodes_);
		archive & boost::serialization::make_nvp("Edges", edge_vector);
	}
	template <class Archive>
	void load(Archive& archive, const unsigned int version) {
		std::vector<std::vector<int> > edge_vector;
		archive & boost::serialization::make_nvp("Action", action_name_);
		archive & boost::serialization::make_nvp("Figure", figure_name_);
		archive & boost::serialization::make_nvp("ShapeContext_Point_Num", sc_point_num_);
		archive & boost::serialization::make_nvp("ShapeContext_Angle_Bin_Num", sc_angle_bin_num_);
		archive & boost::serialization::make_nvp("ShapeContext_Radius_Bin_Num", sc_radius_bin_num_);
		archive & boost::serialization::make_nvp("Nodes", nodes_);
		archive & boost::serialization::make_nvp("Edges", edge_vector);
		tcpp::Convert<int, Eigen::Dynamic, Eigen::Dynamic>(edge_vector, edges_);
	}
#endif
};

} /* namespace prml */
} /* namespace tcpp */

#endif /* TCPP_ACTION_NET_HPP_ */
