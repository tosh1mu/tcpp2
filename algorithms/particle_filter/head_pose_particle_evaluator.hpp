/**
 * @file head_pose_particle_evaluator.hpp
 * @brief Evaluator of head pose particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVALUATOR_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVALUATOR_HPP_

#include "particle_evaluator_interface.hpp"
#include "head_pose_particle.hpp"

#include <tcpp2/vision/image_classifiers/lbp_svm_classifier.hpp>

#include <cassert>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleEvaluator: public ParticleEvaluatorInterface {
public:
	/* constructor, copy, destructor */
	HeadPoseParticleEvaluator(
		int x_min, int x_max, int y_min, int y_max,
		int s_min, int s_max, int d_min, int d_max,
		int lbp_rows, int lbp_cols,
		const std::string& libsvm_model_file, const std::string& libsvm_scale_file,
		int resize_width, int resize_height
	):
		constraint_( x_min, x_max, y_min, y_max, s_min, s_max, d_min, d_max ),
		classifier_( lbp_rows, lbp_cols, libsvm_model_file, libsvm_scale_file )
		{
			classifier_.SetResizeSize( resize_width, resize_height );
		}

	/* method */
	bool Validate( HeadPoseParticle& head_pose_particle )
		{
			if( head_pose_particle.x() >= constraint_.x_min
				&& head_pose_particle.x() + head_pose_particle.s() <= constraint_.x_max
				&& head_pose_particle.y() >= constraint_.y_min
				&& head_pose_particle.y() + head_pose_particle.s() <= constraint_.y_max
			)
				{
					return false;
				}
			else
				{
					int d_cycle = constraint_.d_max - constraint_.d_min + 1;
					int d_correct = head_pose_particle.d();
					while( d_correct < constraint_.d_min )
						{
							d_correct += d_cycle;
						}
					while( d_correct > constraint_.d_max )
						{
							d_correct -= d_cycle;
						}
					assert( d_correct >= constraint_.d_min && d_correct <= constraint_.d_max );
					head_pose_particle.set_d( d_correct );
					return true;
				}
		}

private:
	/* typedef, struct */
	struct Constraint {
		Constraint(
			int _x_min, int _x_max, int _y_min, int _y_max, int _s_min, int _s_max, int _d_min, int _d_max
		)
			{
				x_min = _x_min; x_max = _x_max;
				y_min = _y_min; y_max = _y_max;
				s_min = _s_min; s_max = _s_max;
				d_min = _d_min; d_max = _d_max;
			}
		int x_min, x_max, y_min, y_max, s_min, s_max, d_min, d_max;
	};

	/* variable */
	Constraint constraint_;
	tcpp::vision::LbpSvmClassifier classifier_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
