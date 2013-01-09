/**
 * @file head_pose_particle_eval_base.hpp
 * @brief Evaluation base class of HeadPoseParticle
 * @author Toshimitsu Takahashi
 * @date 2013/1/6
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_HPP_

#include "particle_eval_base_interface.hpp"

#include <tcpp2/vision/types/roi.hpp>

#include <cassert>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleEvalBase: public ParticleEvalBaseInterface
{
public:
	HeadPoseParticleEvalBase( const cv::Mat& image, int offset_x, int offset_y ):
		image_(image), offset_x_(offset_x), offset_y_(offset_y)
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

private:
	const cv::Mat& image_;
	int offset_x_, offset_y_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_HPP_ */
