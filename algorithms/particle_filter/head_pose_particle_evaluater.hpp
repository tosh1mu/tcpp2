/**
 * @file head_pose_particle_evaluater.hpp
 * @brief Evaluater of head pose particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_

#include "particle_evaluater_interface.hpp"
#include "head_pose_particle.hpp"

#include <cassert>
#include <libsvm/svm.h>
#include <opencv2/core/core.hpp>

#include <tcpp2/vision/feature_extractors/feature_extractor_interface.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T>
class HeadPoseParticleEvaluater: public ParticleEvaluaterInterface {
	HeadPoseParticleEvaluater( const cv::Mat& image, int dir_label_start, int dir_label_end ):
		image_(image),
		dir_label_start_(dir_label_start), dir_label_end_(dir_label_end) {}

	bool Validate( HeadPoseParticle& head_pose_particle ) {
		int dir_label_cycle = dir_label_end_ - dir_label_start_ + 1;
		int dir_label = head_pose_particle.d();
		while( dir_label < dir_label_start_ ) {
			dir_label += dir_label_cycle;
		}
		while( dir_label > dir_label_end_ ) {
			dir_label -= dir_label_cycle;
		}
		assert( dir_label >= dir_label_start_ && dir_label <= dir_label_end_ );
		head_pose_particle.set_d( dir_label );
		return ( head_pose_particle.x() >= 0 && head_pose_particle.x() + head_pose_particle.s() <= image_.cols
				 && head_pose_particle.y() >= 0 && head_pose_particle.y() + head_pose_particle.s() <= image_.rows );
	}

	double Likelihood( const HeadPoseParticle& head_pose_particle ) {
	}

private:
	const cv::Mat& image_;
	int dir_label_start_, dir_label_end_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
