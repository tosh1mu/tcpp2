/**
 * @file head_pose_particle_evaluator_m.hpp
 * @brief Evaluator of head pose particle (with model)
 * @author Toshimitsu Takahashi
 * @date 2013/2/2
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_EVALUATOR_M_HPP_
#define TCPP_HEAD_POSE_PARTICLE_EVALUATOR_M_HPP_

#include "tcpp2/algorithms/particle_filter/particle_evaluator_interface.hpp"
#include "head_pose_particle.hpp"
#include "head_pose_particle_eval_base_m.hpp"

#include "tcpp2/core/macros.hpp"
#include "tcpp2/math/geometry_funcs.hpp"
#include "tcpp2/math/probability.hpp"
#include "tcpp2/vision/image_classifiers/image_classifier_interface.hpp"

#include <cassert>
#include <opencv2/core/core.hpp>

#ifdef USING_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range3d.h>
#endif /* USING_TBB */

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HeadPoseParticleEvaluatorM:
		public tcpp::ParticleEvaluatorInterface<HeadPoseParticle, HeadPoseParticleEvalBaseM> {
public:
	/* constructor, copy, destructor */
	HeadPoseParticleEvaluatorM( int s_min, int s_max, int d_num,
								tcpp::vision::ImageClassifierInterface& classifier,
								const tcpp::Discrete2dNormalParams& head_dist_params ):
		s_min_(s_min), s_max_(s_max), d_num_(d_num),
		classifier_( classifier ), head_dist_params_(head_dist_params) {}

	/* method */
	bool Validate( HeadPoseParticle& head_pose_particle,
				   const HeadPoseParticleEvalBaseM& eval_base )
		{
			CorrectDirection( head_pose_particle );
			cv::Rect_<int> image_rect( eval_base.offset_x(), eval_base.offset_y(),
									   eval_base.image().cols, eval_base.image().rows );
			cv::Point2i particle_lu( head_pose_particle.x(), head_pose_particle.y() );
			cv::Point2i particle_rl( head_pose_particle.x() + head_pose_particle.s() - 1,
									 head_pose_particle.y() + head_pose_particle.s() - 1 );
			return ( image_rect.contains( particle_lu ) &&
					 image_rect.contains( particle_rl ) &&
					 head_pose_particle.s() >= s_min_ && head_pose_particle.s() <= s_max_ );
		}

	double Likelihood( const HeadPoseParticle& particle,
					   const HeadPoseParticleEvalBaseM& eval_base )
		{
			std::map<int, double> probabilities;
			GetProbabilities( eval_base, particle.x(), particle.y(), particle.s(), probabilities );
			double log_likelihood = 0.0;
			double dir_prob = probabilities[ particle.d() ];
			double model_prob = GetProbWithModel( eval_base, particle.x(), particle.y(), particle.s() );
			assert( !isnan(dir_prob) && !isnan(model_prob) );

			if( dir_prob > 1E-10 && model_prob > 1E-10 ) {
				log_likelihood = log( dir_prob ) + log( model_prob );
				if( log_likelihood > -10 ) {
					double likelihood = exp( log_likelihood );
					assert( !isnan( likelihood ) );
					return likelihood;
				} else {
					return 0.0;
				}
			} else {
				return 0.0;
			}
		}

	double GetBestParticle( const HeadPoseParticleEvalBaseM& eval_base,
							 HeadPoseParticle& best_particle)
		{
			double max_probability = -1;
			FindBestParticle find_best_particle( *this, eval_base, max_probability, best_particle );
#ifdef USING_TBB
			tbb::blocked_range3d<int, int, int> range(
				s_min_, s_max_ + 1, 10,
				eval_base.offset_x(), eval_base.offset_x() + eval_base.image().cols, 10,
				eval_base.offset_y(), eval_base.offset_y() + eval_base.head_area_height(), 10 );
			tbb::parallel_for( range, find_best_particle );
#else /* ifdef USING_TBB */
			find_best_particle();
#endif /* ifdef USING_TBB */
			return max_probability;
		}

private:
	/* typedef, struct */

	/* Methods */
	void CorrectDirection( HeadPoseParticle& head_pose_particle )
		{
			int direction = head_pose_particle.d();
			while( direction < 0 ) {
				direction += d_num_;
			}
			while( direction >= d_num_ ) {
				direction -= d_num_;
			}
			assert( direction >= 0 && direction < d_num_ );
			head_pose_particle.set_d( direction );
		}

	void GetProbabilities( const HeadPoseParticleEvalBaseM& eval_base,
						   int x, int y, int s, std::map<int, double>& probabilities )
		{
			cv::Rect_<int> roi_rect( x - eval_base.offset_x(), y - eval_base.offset_y(), s, s );
			cv::Mat roi_image = eval_base.image()( roi_rect );
			classifier_.PredictProbability( roi_image, probabilities );
		}

	double GetProbWithModel( const HeadPoseParticleEvalBaseM& eval_base,
							 int x, int y, int s )
		{
			cv::Point2i model_top = eval_base.model().top();
			model_top.x += eval_base.offset_x();
			model_top.y += eval_base.offset_y();
			Eigen::Vector2i vec;
			double min_dist = 1E10;
			for( int i = 0; i < s; ++i ) {
				int relative_x = (x + i) - model_top.x;
				int relative_y = (y + s - 1) - model_top.y;
				double dist = static_cast<double>( relative_x * relative_x + relative_y * relative_y );
				dist = sqrt(dist);
				if( dist < min_dist ) {
					min_dist = dist;
					vec.coeffRef(0) = relative_x;
					vec.coeffRef(1) = relative_y;
				}
			}
			double p = tcpp::Discrete2dNormalPdf<double>( vec, head_dist_params_ );
			return p;
		}

	class FindBestParticle {
	public:
		FindBestParticle( HeadPoseParticleEvaluatorM& evaluator,
						  const HeadPoseParticleEvalBaseM& eval_base,
						  double& max_probability,
						  HeadPoseParticle& best_particle ):
			evaluator_(evaluator), eval_base_(eval_base),
			max_probability_(max_probability), best_particle_(best_particle) {}

#ifdef USING_TBB
		void operator()( const tbb::blocked_range3d<int, int, int>& range ) const
			{
				for( int s = range.pages().begin(); s != range.pages().end(); ++s ) {
					for( int x = range.rows().begin(); x != range.rows().end() &&
							 x < eval_base_.offset_x() + eval_base_.head_area_height() - s - 1; ++x ) {
						for( int y = range.cols().begin(); y != range.cols().end() &&
								 y < eval_base_.offset_y() + eval_base_.image().rows - s - 1; ++y ) {
							int best_direction;
							double probability = FindBestDirection( x, y, s, eval_base_, best_direction );
							if( probability > max_probability_ ) {
								HeadPoseParticle particle( x, y, s, best_direction );
								best_particle_ = particle;
								max_probability_ = probability;
							}
						}
					}
				}
			}
#endif /* USING_TBB */

		void operator()() const
			{
				for( int s = evaluator_.s_min_; s <= evaluator_.s_max_; ++s ) {
					for( int x = eval_base_.offset_x();
						 x < eval_base_.offset_x() + eval_base_.image().cols - s; ++x ) {
						for( int y = eval_base_.offset_y();
							 y < eval_base_.offset_y() + eval_base_.head_area_height() - s; ++y ) {
							int best_direction;
							double probability = FindBestDirection( x, y, s, eval_base_, best_direction );
							if( probability > max_probability_ ) {
								HeadPoseParticle particle( x, y, s, best_direction );
								best_particle_ = particle;
								max_probability_ = probability;
							}
						}
					}
				}
			}

	private:
		double FindBestDirection( int x, int y, int s,
								  const HeadPoseParticleEvalBaseM& eval_base,
								  int& best_direction ) const
			{
				std::map<int, double> probabilities;
				evaluator_.GetProbabilities( eval_base, x, y, s, probabilities );

				best_direction = -1;
				double max_prob = 0.0;
				for( typename std::map<int, double>::const_iterator prob_itr = probabilities.begin();
					 prob_itr != probabilities.end(); ++prob_itr ) {
					int label = prob_itr->first;
					double prob = prob_itr->second;
					if( label > -1 && prob > max_prob ) {
						best_direction = label;
						max_prob = prob;
					}
				}
				return max_prob;
			}
		
		HeadPoseParticleEvaluatorM& evaluator_;
		const HeadPoseParticleEvalBaseM& eval_base_;
		double& max_probability_;
		HeadPoseParticle& best_particle_;
	};

	/* variable */
	int s_min_, s_max_, d_num_;
	tcpp::vision::ImageClassifierInterface& classifier_;
	const tcpp::Discrete2dNormalParams& head_dist_params_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_M_HPP_ */
