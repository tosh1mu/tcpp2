/*
 * shape_context.hpp
 *
 *  Created on: 2012/09/26
 *      Author: takahashi
 */

#ifndef TCPP_SHAPECONTEXT_HPP_
#define TCPP_SHAPECONTEXT_HPP_

#include "../../core/core.hpp"
#include "../../core/convert.hpp"
#include "../../core/math/random.hpp"
#include "../../core/math/histogram.hpp"
#include "../../mp/combination/lapjv.hpp"

#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef USING_BOOST_SERIALIZATION
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#endif

namespace tcpp {
namespace ip {

class ShapeContext {
public:
	/* constructor(s) and destructor(s) */
	ShapeContext():
		point_num_(0), angle_bin_num_(0), radius_bin_num_(0){}
	ShapeContext(const int &point_num, const int &angle_bin_num, const int &radius_bin_num):
		point_num_(point_num), angle_bin_num_(angle_bin_num), radius_bin_num_(radius_bin_num),
		histograms_(point_num, Eigen::ArrayXi::Zero(radius_bin_num*angle_bin_num)){}
	virtual ~ShapeContext(){}

	/* public methods */
	void Extract(const cv::Mat &image) {
		std::vector<Eigen::Vector2i> edge_points;
		std::vector<std::vector<Eigen::Vector2i> > relative_vectors_sets(point_num_);
		std::vector<double> lengths;
		lengths.reserve(point_num_*point_num_);

		GetEdgePoints(image, edge_points);
		boost::accumulators::accumulator_set< double, boost::accumulators::features<boost::accumulators::tag::median> > acc;
		for(int i=0 ; i<point_num_ ; ++i) {
			CalcRelativeVectors(edge_points, i, relative_vectors_sets[i], lengths);
			for( int j=0 ; j<static_cast<int>(lengths.size()) ; ++j )
				acc(lengths[j]);
		}
		double median = boost::accumulators::extract::median(acc);
		CalcHistograms(relative_vectors_sets, median);
	}

	/* accessors */
	const std::vector<Eigen::ArrayXi>& histograms() const {return histograms_;}
private:
	/* private variables */
	int point_num_, angle_bin_num_, radius_bin_num_;
	std::vector<Eigen::ArrayXi> histograms_;

	/* private methods */
	void GetEdgePoints(const cv::Mat &image, std::vector<Eigen::Vector2i> &edge_points) {
		int rows = image.rows;
		int cols = image.cols;

		cv::Mat edge_image;
		cv::Canny(image, edge_image, 50, 200);

		int edge_point_num = cv::countNonZero(edge_image);
		double scale = sqrt(static_cast<double>(point_num_) / static_cast<double>(edge_point_num));
		int scaled_rows = static_cast<int>(ceil(rows*scale));
		int scaled_cols = static_cast<int>(ceil(cols*scale));

		cv::Mat scaled_edge_image(scaled_rows, scaled_cols, edge_image.type());
		cv::resize(edge_image, scaled_edge_image, scaled_edge_image.size(), cv::INTER_AREA);

		std::map<double, Eigen::Vector2i> scaled_edge_map;
		tcpp::RandNumMaker<boost::uniform_01<> > rand01;
		for(int row=0 ; row<scaled_rows ; ++row) {
			uchar *p = scaled_edge_image.ptr<uchar>(row);
			for(int col=0 ; col<scaled_cols ; ++col) {
				Eigen::Vector2i edge_point;
				edge_point << row, col;
				int a = cv::saturate_cast<int>(p[col]);
				scaled_edge_map[static_cast<double>(a) + rand01()] = edge_point;
			}
		}
		assert( static_cast<int>( scaled_edge_map.size() ) >= point_num_);
		if(!edge_points.empty())
			edge_points.clear();
		edge_points.reserve(point_num_);
		std::map<double, Eigen::Vector2i>::reverse_iterator scaled_edge_map_itr = scaled_edge_map.rbegin();
		for(int i=0 ; i<point_num_ ; ++i, ++scaled_edge_map_itr)
			edge_points.push_back(scaled_edge_map_itr->second);
	}

	void CalcRelativeVectors(const std::vector<Eigen::Vector2i> &edge_points, const int &index, std::vector<Eigen::Vector2i> &relative_vectors, std::vector<double> &lengths) {
		assert( index < point_num_ );
		relative_vectors.reserve(point_num_-1);
		for(int i=0 ; i<point_num_ ; ++i) {
			if( i==index )
				continue;
			else {
				Eigen::Vector2i relative_vector = edge_points[i] - edge_points[index];
				relative_vectors.push_back(relative_vector);
				double length = sqrt(relative_vector.dot( relative_vector ));
				lengths.push_back(length);
			}
		}
	}

	void Vote(const Eigen::Vector2i &relative_vector, const double &median, Eigen::ArrayXi &histogram) {
		double angle = atan2(static_cast<double>(relative_vector(1)), static_cast<double>(relative_vector(0))) + M_PI;
		if(angle == 2*M_PI)
			angle = 0;
		double length = sqrt(relative_vector.dot(relative_vector)) / median;

		int angle_bin = static_cast<int>(floor(angle*angle_bin_num_/(2*M_PI)));
		int radius_bin = static_cast<int>(floor(log(length))) + radius_bin_num_/2 + radius_bin_num_%2;
		if(radius_bin<0)
			radius_bin = 0;
		else if(radius_bin>=radius_bin_num_)
			radius_bin = radius_bin_num_-1;
		int bin = angle_bin*radius_bin_num_ + radius_bin;
		histogram(bin) += 1;
	}

	void CalcHistograms(const std::vector<std::vector<Eigen::Vector2i> > &relative_vectors_sets, const double &median) {
		for(int pt=0 ; pt<point_num_ ; ++pt) {
			for(int i=0 ; i<static_cast<int>(relative_vectors_sets[pt].size()) ; ++i) {
				Eigen::Vector2i vec = relative_vectors_sets[pt][i];
				Vote(vec, median, histograms_[pt]);
			}
		}
	}

#ifdef USING_BOOST_SERIALIZATION
	/* boost::serialization methods */
	friend class boost::serialization::access;
	BOOST_SERIALIZATION_SPLIT_MEMBER();

	template <class Archive>
	void save(Archive& archive, const unsigned int version) const {
		std::vector<std::vector<int> > histogram_vectors;
		histogram_vectors.reserve(point_num_);
		for(int i=0 ; i<point_num_ ; ++i) {
			std::vector<int> histogram_vector;
			tcpp::Convert<int, Eigen::Dynamic>(histograms_[i], histogram_vector);
			histogram_vectors.push_back(histogram_vector);
		}
		archive & boost::serialization::make_nvp("Point_Num", point_num_);
		archive & boost::serialization::make_nvp("Angle_Bin_Num", angle_bin_num_);
		archive & boost::serialization::make_nvp("Radius_Bin_Num", radius_bin_num_);
		archive & boost::serialization::make_nvp("Histograms", histogram_vectors);
	}

	template <class Archive>
	void load(Archive& archive, const unsigned int version) {
		std::vector<std::vector<int> > histogram_vectors;
		archive & boost::serialization::make_nvp("Point_Num", point_num_);
		archive & boost::serialization::make_nvp("Angle_Bin_Num", angle_bin_num_);
		archive & boost::serialization::make_nvp("Radius_Bin_Num", radius_bin_num_);
		archive & boost::serialization::make_nvp("Histograms", histogram_vectors);
		if(!histograms_.empty())
			histograms_.clear();
		histograms_.reserve(point_num_);
		for(int i=0 ; i<point_num_ ; ++i) {
			Eigen::ArrayXi histogram;
			tcpp::Convert<int, Eigen::Dynamic>(histogram_vectors[i], histogram);
			histograms_.push_back(histogram);
		}
	}
#endif
};

inline double Match(const ShapeContext &sc1, const ShapeContext &sc2) {
	Eigen::ArrayXXd cost_array;
	GetDistanceArray(sc1.histograms(), sc2.histograms(), cost_array);
	tcpp::mp::LAPJV lap(cost_array);
	double cost = lap.Solve(2);
	return cost;
}

inline void Convert(const std::vector<cv::Mat>& image_sequence, const int& point_num, const int& angle_bin_num, const int&radius_bin_num, std::vector<ShapeContext>& sc_sequence) {
	sc_sequence.reserve(image_sequence.size());
	for(unsigned int i=0 ; i<image_sequence.size() ; ++i) {
		ShapeContext shape_context(point_num, angle_bin_num, radius_bin_num);
		cv::Mat gray_img;
		cv::cvtColor(image_sequence[i], gray_img, CV_BGR2GRAY);
		shape_context.Extract(gray_img);
		sc_sequence.push_back(shape_context);
	}
}

} /* namespace ip */
} /* namespace tcpp */

#endif /* TCPP_SHAPECONTEXT_HPP_ */
