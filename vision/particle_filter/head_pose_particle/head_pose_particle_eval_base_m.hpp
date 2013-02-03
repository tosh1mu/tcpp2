/**
 * @file head_pose_particle_eval_base_m.hpp
 * @brief Evaluation base class (with model) of HeadPoseParticle
 * @author Toshimitsu Takahashi
 * @date 2013/2/2
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_M_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_M_HPP_

#include "tcpp2/algorithms/particle_filter/particle_eval_base_interface.hpp"
#include "tcpp2/vision/types/roi.hpp"
#include "tcpp2/vision/tmp/body_line_model.hpp"

#include <cassert>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HeadPoseParticleEvalBaseM: public tcpp::ParticleEvalBaseInterface
{
public:
	HeadPoseParticleEvalBaseM( const cv::Mat& image, int offset_x, int offset_y, double head_area_height_rate,
							  const BodyLineModel& model ):
		image_(image), offset_x_(offset_x), offset_y_(offset_y), head_area_height_rate_(head_area_height_rate),
		model_(model)
		{
			assert( image_.rows > 0 && image_.cols > 0 );
		}

	bool IsSet() const
		{
			return image_.rows > 0 && image_.cols > 0;
		}

	const cv::Mat& image() const { return image_; }
	int offset_x() const { return offset_x_; }
	int offset_y() const { return offset_y_; }
	int head_area_height() const { return static_cast<int>( floor( image_.rows * head_area_height_rate_ + 0.5 ) ); }
	const BodyLineModel& model() const { return model_; }
private:
	const cv::Mat& image_;
	int offset_x_, offset_y_;
	double head_area_height_rate_;
	const BodyLineModel& model_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_M_HPP_ */
