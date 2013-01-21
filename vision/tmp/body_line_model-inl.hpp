/**
 * @file body_line_model-inl.hpp
 * @brief BRIEF_EXPLANATION
 * @author takahashi
 * @date 2013/01/20
 * @version 0.0.1
 */

#ifndef TCPP_BODY_LINE_MODEL_INL_HPP_
#define TCPP_BODY_LINE_MODEL_INL_HPP_

#include "body_line_model.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <prob.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tcpp2/math/geometry_funcs.hpp>

namespace tcpp {
namespace vision {

namespace body_line_model {

int GetRowCentroid( const cv::Mat& row )
{
	assert( row.rows == 1 && row.type() == CV_32FC1 );
	double weighted_mean = 0.0;
	double denominator = 0.0;
	for( int col = 0, n = row.cols; col < n; ++col ) {
		double val = cv::saturate_cast<double>( row.at<float>(col) );
		double weighted_col = val * static_cast<double>(col);
		if( weighted_col > 1E-03 ) {
			weighted_mean += weighted_col;
			denominator += val;
		}
	}
	if( denominator > 0.0 ) {
		weighted_mean /= denominator;
		return static_cast<int>( floor( weighted_mean + 0.5 ) );
	} else {
		return -1;
	}
}

void GetLineParams( int x0, int y0, int x1, int y1, double& length, double& angle )
{
	double dx = static_cast<double>( x1 - x0 );
	double dy = static_cast<double>( y1 - y0 );
	length = sqrt( dx*dx + dy*dy );
	angle = atan2( dy, dx );
	if( angle < 0 )
		angle += M_PI;
}

} /* namespace body_line_model */

using namespace tcpp::vision::body_line_model;

double BodyLineModel::Joint::LogLikelihood( const Line& parent, const Line& child ) const
{
	double length_rate = child.length() / parent.length();
	double angle = child.angle() - parent.angle();

	double length_rate_prob_norm_const = normal_pdf( length_rate_mean_, length_rate_mean_, length_rate_stddev_ );
	double angle_prob_norm_const = von_mises_pdf( angle_mean_, angle_mean_, angle_beta_ );

	double length_rate_prob = normal_pdf( length_rate, length_rate_mean_, length_rate_stddev_ );
	length_rate_prob /= length_rate_prob_norm_const;
	double angle_prob = von_mises_pdf( angle, angle_mean_, angle_beta_ );
	angle_prob /= angle_prob_norm_const;

	return log( length_rate_prob ) + log( angle_prob );
}

double BodyLineModel::MatchLikelihood( const cv::Mat& silhouette ) const
{
	std::vector<int> centroids, norm_centroids;
	GetCentroids( silhouette, centroids );
	NormalizeCentroids( centroids, normalizing_window_radius_, norm_centroids );

	double normalizing_const = 1.0 / normal_pdf( 0, 0, match_stddev_ );
	double sum_likelihood = 0.0;
	double point_num = 0.0;

	for( int i = 0, n = static_cast<int>( norm_centroids.size() ); i < n; ++i ) {
		if( norm_centroids[i] >= 0 ) {
			double distance = GetDistance( norm_centroids[i], i );
			double prob = normal_pdf( distance, 0, match_stddev_ );
			sum_likelihood += prob * normalizing_const;
			point_num += 1.0;
		}
	}

	return sum_likelihood / point_num;
}

double BodyLineModel::DeformLikelihood() const
{
	double lower_back_loglh = lower_back_.LogLikelihood( body_, thigh_ );
	double knee_loglh = knee_.LogLikelihood( thigh_, calf_ );
	double log_lh = lower_back_loglh + knee_loglh;
	if( log_lh < -10.0 ) {
		return 0.0;
	} else {
		return exp( log_lh);
	}
}

void BodyLineModel::GetPoints( std::vector<cv::Point2i>& points ) const
{
	assert( points.empty() );
	points.reserve( 4 );
	SegmentParams body_seg, thigh_seg, calf_seg;
	GetBodySegment( body_seg );
	GetThighSegment( body_seg, thigh_seg );
	GetCalfSegment( thigh_seg, calf_seg );
	cv::Point2i top( body_seg.x0, body_seg.y0 );
	points.push_back( top );
	cv::Point2i lower_back( body_seg.x1, body_seg.y1 );
	points.push_back( lower_back );
	cv::Point2i knee( thigh_seg.x1, thigh_seg.y1 );
	points.push_back( knee );
	cv::Point2i end( calf_seg.x1, calf_seg.y1 );
	points.push_back( end );
}

void BodyLineModel::GetCentroids( const cv::Mat& silhouette, std::vector<int>& centroids ) const
{
	assert( silhouette.type() == CV_8UC1 );
	assert( centroids.empty() );
	cv::Mat distance_image;
	cv::distanceTransform( silhouette, distance_image, CV_DIST_L2, 5 );
	cv::normalize( distance_image, distance_image, 0.0, 1.0, CV_MINMAX );
	centroids.reserve( distance_image.rows );
	for( int r = 0, n = distance_image.rows; r < n; ++r ) {
		cv::Mat row = distance_image.row(r);
		int center = GetRowCentroid( row );
		centroids.push_back( center );
	}
}

void BodyLineModel::NormalizeCentroids( const std::vector<int>& centroids,
		int window_radius, std::vector<int>& norm_centroids ) const
{
	assert( norm_centroids.empty() );
	int rows = static_cast<int>( centroids.size() );
	norm_centroids.reserve( rows );
	for( int r = 0; r < rows; ++r ) {
		int begin_ind, end_ind;
		if( r < window_radius ) {
			begin_ind = 0;
		} else {
			begin_ind = r - window_radius;
		}
		if( r > rows - 1 - window_radius ) {
			end_ind = rows;
		} else {
			end_ind = r + window_radius;
		}

		double normalized_col = 0.0;
		double valid_num = 0.0;
		for( int i = begin_ind; i <= end_ind; ++i ) {
			if( centroids[i] >= 0 ) {
				normalized_col += static_cast<double>( centroids[i] );
				valid_num += 1.0;
			}
		}

		if( valid_num > 0.0 ) {
			normalized_col /= valid_num;
			norm_centroids.push_back( static_cast<int>( floor( normalized_col + 0.5 ) ) );
		} else {
			norm_centroids.push_back( -1 );
		}
	}
}

void BodyLineModel::EstimateFromCentroids( const std::vector<int>& centroids )
{
	int rows = static_cast<int>( centroids.size() );
	std::vector<cv::Point2i> centroid_coordinates;
	centroid_coordinates.reserve( rows );
	for( int r = 0; r < rows; ++r ) {
		if( centroids[r] > 0 ) {
			centroid_coordinates.push_back( cv::Point2i( centroids[r], r ) );
		}
	}

	double height = static_cast<double>( centroid_coordinates.size() ) * 0.9;
	int neck_ind = static_cast<int>( floor( height * 0.19 + 0.5 ) );
	int lower_back_ind = static_cast<int>( floor( height * 0.49 + 0.5 ) );
	int knee_ind = static_cast<int>( floor( height * 0.74 + 0.5 ) );
	int foot_ind = static_cast<int>( floor( height + 0.5 ) );

	x_ = ( centroid_coordinates[neck_ind].x + centroid_coordinates[lower_back_ind].x ) * 0.5;
	y_ = ( centroid_coordinates[neck_ind].y + centroid_coordinates[lower_back_ind].y ) * 0.5;

	double body_length, body_angle;
	GetLineParams( centroid_coordinates[neck_ind].x, centroid_coordinates[neck_ind].y,
			centroid_coordinates[lower_back_ind].x, centroid_coordinates[lower_back_ind].y, body_length, body_angle );
	body_ = Line( body_length, body_angle );

	double thigh_length, thigh_angle;
	GetLineParams( centroid_coordinates[lower_back_ind].x, centroid_coordinates[lower_back_ind].y,
			centroid_coordinates[knee_ind].x, centroid_coordinates[knee_ind].y, thigh_length, thigh_angle );
	thigh_ = Line( thigh_length, thigh_angle );

	double calf_length, calf_angle;
	GetLineParams( centroid_coordinates[knee_ind].x, centroid_coordinates[knee_ind].y,
			centroid_coordinates[foot_ind].x, centroid_coordinates[foot_ind].y, calf_length, calf_angle );
	calf_ = Line( calf_length, calf_angle );
}

void BodyLineModel::GetBodySegment( SegmentParams& params ) const {
	double dx = cos( body_.angle() );
	double dy = sin( body_.angle() );

	params.x0 = x_ - dx * body_.length() * 0.5;
	params.y0 = y_ - dy * body_.length() * 0.5;
	params.x1 = params.x0 + dx * body_.length();
	params.y1 = params.y0 + dy * body_.length();
}

void BodyLineModel::GetThighSegment( const SegmentParams& body_params, SegmentParams& params ) const
{
	double dx = cos( thigh_.angle() );
	double dy = sin( thigh_.angle() );

	params.x0 = body_params.x1;
	params.y0 = body_params.y1;
	params.x1 = params.x0 + dx * thigh_.length();
	params.y1 = params.y0 + dy * thigh_.length();
}

void BodyLineModel::GetCalfSegment( const SegmentParams& thigh_params, SegmentParams& params ) const
{
	double dx = cos( calf_.angle() );
	double dy = sin( calf_.angle() );

	params.x0 = thigh_params.x1;
	params.y0 = thigh_params.y1;
	params.x1 = params.x0 + dx * calf_.length();
	params.y1 = params.y0 + dy * calf_.length();
}

double BodyLineModel::GetDistance( int x, int y ) const
{
	SegmentParams body_seg, thigh_seg, calf_seg;
	GetBodySegment( body_seg );
	GetThighSegment( body_seg, thigh_seg );
	GetCalfSegment( thigh_seg, calf_seg );

	double min_distance = tcpp::PointSegmentDistance( body_seg.x0, body_seg.y0, body_seg.x1, body_seg.y1, x, y );
	min_distance = std::min( min_distance, tcpp::PointSegmentDistance( thigh_seg.x0, thigh_seg.y0, thigh_seg.x1, thigh_seg.y1, x, y ) );
	min_distance = std::min( min_distance, tcpp::PointSegmentDistance( calf_seg.x0, calf_seg.y0, calf_seg.x1, calf_seg.y1, x, y ) );
	return min_distance;
}

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_BODY_LINE_MODEL_INL_HPP_ */
