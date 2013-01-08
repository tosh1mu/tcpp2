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
	HeadPoseParticleEvalBase( const cv::Mat& image ): image_(image)
		{
			assert( image_.rows > 0 && image_.cols > 0 );
		}

	bool IsSet() const
		{
			return image_.rows > 0 && image_.cols > 0;
		}

	const cv::Mat& image() const { return image_; }

private:
	const cv::Mat& image_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVAL_BASE_HPP_ */
