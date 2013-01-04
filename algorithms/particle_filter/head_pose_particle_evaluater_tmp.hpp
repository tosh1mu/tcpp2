/**
 * @file head_pose_particle_evaluater_tmp.hpp
 * @brief Temporary definition of evaluater of head pose particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVALUATER_TMP_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVALUATER_TMP_HPP_

#include "head_pose_particle.hpp"

#include <cassert>
#include <libsvm/svm.h>
#include <opencv2/core/core.hpp>

#include <tcpp2/vision/image_classifiers/lbp_svm_tmp.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleEvaluaterTmp {
public:
	HeadPoseParticleEvaluaterTmp( int dir_label_start, int dir_label_end,
								  int lbp_rows, int lbp_cols,
								  const std::string& libsvm_model_file, const std::string& libsvm_scale_file,
								  int resize_width, int resize_height ):
		dir_label_start_(dir_label_start), dir_label_end_(dir_label_end),
		image_classifier_(lbp_rows, lbp_cols, libsvm_model_file, libsvm_scale_file)
		{
			image_classifier_.SetResizeSize( resize_width, resize_height );
		}

	bool Validate( const cv::Mat& image, int offset_x, int offset_y, HeadPoseParticle& head_pose_particle ) {
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
		return ( head_pose_particle.x() >= offset_x
				 && head_pose_particle.x() + head_pose_particle.s() <= image.cols + offset_x
				 && head_pose_particle.y() >= offset_y
				 && head_pose_particle.y() + head_pose_particle.s() <= image.rows + offset_y
		);
	}

	double Likelihood( const cv::Mat& image, int offset_x, int offset_y, const HeadPoseParticle& head_pose_particle ) {
		cv::Rect_<int> roi_rect( head_pose_particle.x() - offset_x, head_pose_particle.y() - offset_y,
								 head_pose_particle.s(), head_pose_particle.s() );
		cv::Mat roi_image( image, roi_rect );
		std::map<int, double> probs;
		image_classifier_.PredictProb( roi_image, probs );
		assert( probs.count( head_pose_particle.d() ) == 1 );
		return probs[head_pose_particle.d()];
	}

private:
	int dir_label_start_, dir_label_end_;
	tcpp::vision::LBPSVMTmp image_classifier_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_TMP_HPP_ */
