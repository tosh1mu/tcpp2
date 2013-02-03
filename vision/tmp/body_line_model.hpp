/**
 * @file body_line_model.hpp
 * @brief BRIEF_EXPLANATION
 * @author takahashi
 * @date 2013/01/20
 * @version 0.0.1
 */

#ifndef TCPP_BODY_LINE_MODEL_HPP_
#define TCPP_BODY_LINE_MODEL_HPP_

#include <cassert>
#include <opencv2/core/core.hpp>

namespace tcpp {

namespace vision {

class BodyLineModel
{
public:
	/* Subclasses */
	class Line
	{
	public:
		Line(): length_(-1), angle_(-1) {}

		Line( double length, double angle ): length_( length ), angle_( angle ) {}

		Line( const Line& line ): length_( line.length() ), angle_( line.angle() ) {}

		double length() const { return length_; }

		double angle() const { return angle_; }

	private:
		double length_, angle_;
	};

	class Joint
	{
	public:
		Joint(): length_rate_mean_(-1), length_rate_stddev_(-1),
			angle_mean_(-1), angle_beta_(-1) {}

		Joint( double length_rate_mean, double length_rate_stddev, double angle_mean, double angle_beta ):
			length_rate_mean_( length_rate_mean ), length_rate_stddev_( length_rate_stddev ),
			angle_mean_( angle_mean ), angle_beta_( angle_beta ) {}

		Joint( const Joint& joint ):
			length_rate_mean_( joint.length_rate_mean() ), length_rate_stddev_( joint.length_rate_stddev() ),
			angle_mean_( joint.angle_mean() ), angle_beta_( joint.angle_beta() ) {}

		double LogLikelihood( const Line& parent, const Line& child ) const;

		double length_rate_mean() const { return length_rate_mean_; }

		double length_rate_stddev() const { return length_rate_stddev_; }

		double angle_mean() const { return angle_mean_; }

		double angle_beta() const { return angle_beta_; }

	private:
		double length_rate_mean_, length_rate_stddev_; // normal distribution

		double angle_mean_, angle_beta_; // von Mises distribution
	};

	/* Constructors and destructors */
	/**
	 * @brief Default constructor
	 */
	BodyLineModel(): x_(-1), y_(-1), match_stddev_(-1), normalizing_window_radius_(-1) {}

	BodyLineModel( const cv::Mat& silhouette, Joint lower_back, Joint knee, double match_stddev, int normalizing_window_radius ):
		x_(-1), y_(-1), lower_back_(lower_back), knee_(knee), match_stddev_(match_stddev), normalizing_window_radius_(normalizing_window_radius)
	{
		std::vector<int> centroids, norm_centroids;
		GetCentroids( silhouette, centroids );
		NormalizeCentroids( centroids, normalizing_window_radius_, norm_centroids );
		EstimateFromCentroids( norm_centroids );
	}

	BodyLineModel( int x, int y, Line body, Line thigh, Line calf, Joint lower_back, Joint knee, double var_match, int normalizing_window_radius ):
		x_(x), y_(y), body_(body), thigh_(thigh), calf_(calf), lower_back_(lower_back), knee_(knee), match_stddev_(var_match), normalizing_window_radius_(normalizing_window_radius) {}

	double MatchLikelihood( const cv::Mat& silhouette ) const;

	double DeformLikelihood() const;

	void GetPoints( std::vector<cv::Point2i>& points ) const;

	int x() const { return x_; }
	int y() const { return y_; }
	const Line& body() const { return body_; }
	const Line& thigh() const { return thigh_; }
	const Line& calf() const { return calf_; }
	const Joint& lower_back() const { return lower_back_; }
	const Joint& knee() const { return knee_; }
	double var_match() const { return match_stddev_; }

	cv::Point2i top() const {
		SegmentParams body_seg;
		GetBodySegment( body_seg );
		cv::Point2i top( body_seg.x0, body_seg.y0 );
		return top;
	}

private:
	struct SegmentParams {
		int x0, y0, x1, y1;
	};

	/* Methods */
	void GetCentroids( const cv::Mat& silhouette, std::vector<int>& centroids ) const;

	void NormalizeCentroids( const std::vector<int>& centroids, int window_radius, std::vector<int>& norm_centroids ) const;

	void EstimateFromCentroids( const std::vector<int>& centroids );

	void GetBodySegment( SegmentParams& params ) const;

	void GetThighSegment( const SegmentParams& body_params, SegmentParams& params ) const;

	void GetCalfSegment( const SegmentParams& thigh_params, SegmentParams& params ) const;

	double GetDistance( int x, int y ) const;

	/* Data Member */
	// model parameters
	int x_, y_; // coordinate of center point of the body line
	// Parts
	Line body_, thigh_, calf_;
	Joint lower_back_, knee_;

	// matching constraint parameters
	double match_stddev_; // variance of the mean distance between centroid set and the model

	int normalizing_window_radius_;
};

} /* namespace vision */
} /* namespace tcpp */

#include "body_line_model-inl.hpp"

#endif /* TCPP_BODY_LINE_MODEL_HPP_ */
