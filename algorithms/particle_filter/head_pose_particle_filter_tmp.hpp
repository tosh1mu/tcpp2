/**
 * @file head_pose_particle_filter_tmp.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2013/1/4
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_FILTER_TMP_HPP_
#define TCPP_HEAD_POSE_PARTICLE_FILTER_TMP_HPP_

#include "head_pose_particle_generator_tmp.hpp"
#include "head_pose_particle_evaluater_tmp.hpp"

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleFilterTmp {
public:
	HeadPoseParticleFilterTmp( int particle_num,
							   int dir_label_start, int dir_label_end,
							   int lbp_rows, int lbp_cols,
							   const std::string& libsvm_model_file, const std::string& libsvm_scale_file,
							   int resize_width, int resize_height,
							   int sigma_x, int sigma_y, int sigma_s, int sigma_d ):
		particle_num_(particle_num),
		dir_label_start_(dir_label_start), dir_label_end_(dir_label_end),
		evaluater_( dir_label_start, dir_label_end,
					lbp_rows, lbp_cols,
					libsvm_model_file, libsvm_scale_file,
					resize_width, resize_height ),
		generator_( sigma_x, sigma_y, sigma_s, sigma_d ),
		particles_(), particle_weights_()
		{
			particles_.reserve( particle_num_ );
			double init_weight = 1.0 / static_cast<double>( particle_num_ );
			particle_weights_.resize( particle_num_, init_weight );
		}

	void Initialize( const cv::Mat& image, int offset_x, int offset_y,
					 int min_scale, int max_scale, int scale_step, HeadPoseParticle& initial_estimation )
		{
			double max_likelihood = -1E20;
			for( int scale = min_scale; scale <= max_scale; scale += scale_step ) {
				for( int x = offset_x; x < offset_x + image.cols; ++x ) {
					for( int y = offset_y; y < offset_y + image.rows; ++y ) {
						for( int dir_label = dir_label_start_; dir_label <= dir_label_end_; ++dir_label ) {
							HeadPoseParticle particle( x, y, scale, dir_label );
							double likelihood = evaluater_.Likelihood( image, offset_x, offset_y, particle );
							if( likelihood > max_likelihood ) {
								initial_estimation = particle;
								max_likelihood = likelihood;
							}
						}
					}
				}
			}

			for( int i = 0; i < particle_num_; ) {
				HeadPoseParticle particle;
				generator_.Generate( initial_estimation, particle );
				if( evaluater_.Validate( image, offset_x, offset_y, particle ) ) {
					particles_.push_back( particle );
					++i;
					double likelihood = evaluater_.Likelihood( image, offset_x, offset_y, particle );
					particle_weights_[i] *= likelihood;
				}
			}
		}

	void Process( const cv::Mat& image, int offset_x, int offset_y, HeadPoseParticle& estimation )
		{
			
		}

private:
	/* Methods */
	void NormalizeWeights()
		{
			assert( static_cast<int>( particle_weights_.size() ) == particle_num_ );
			double denominator = 0.0;
			for( int i = 0; i < particle_num_; ++i ) {
				denominator += particle_weights_[i];
			}
			for( int i = 0; i < particle_num_; ++i ) {
				particle_weights_[i] /= denominator;
			}
		}

	/* Data members */
	int particle_num_;
	int dir_label_start_, dir_label_end_;
	HeadPoseParticleEvaluaterTmp evaluater_;
	HeadPoseParticleGeneratorTmp generator_;
	std::vector<HeadPoseParticle> particles_;
	std::vector<double> particle_weights_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_FILTER_TMP_HPP_ */
