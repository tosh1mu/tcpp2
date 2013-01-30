/**
 * @file head_body_particle_evaluator.hpp
 * @brief Evaluator of head body particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/30
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_BODY_PARTICLE_EVALUATOR_HPP_
#define TCPP_HEAD_BODY_PARTICLE_EVALUATOR_HPP_

#include "tcpp2/algorithms/particle_filter/particle_evaluator_interface.hpp"
#include "head_body_particle.hpp"
#include "head_body_particle_eval_base.hpp"

#include "tcpp2/vision/image_classifiers/image_classifier_interface.hpp"

#include <cassert>
#include <cmath>
#include <prob.hpp>
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

class HeadBodyParticleEvaluator:
		public tcpp::ParticleEvaluatorInterface<HeadBodyParticle, HeadBodyParticleEvalBase> {
public:
	/* constructor, copy, destructor */
	HeadBodyParticleEvaluator( int s_min, int s_max, int head_dir_num, int body_dir_num,
							   double angle_diff_mean, double angle_diff_beta, double angle_diff_weight,
							   tcpp::vision::ImageClassifierInterface& head_classifier,
							   tcpp::vision::ImageClassifierInterface& body_classifier
	):
		s_min_(s_min), s_max_(s_max), head_dir_num_(head_dir_num), body_dir_num_(body_dir_num),
		angle_diff_mean_(angle_diff_mean), angle_diff_beta_(angle_diff_mean), angle_diff_weight_(angle_diff_weight),
		head_classifier_(head_classifier), body_classifier_(body_classifier) {}

	/* method */
	bool Validate( HeadBodyParticle& head_body_particle,
				   const HeadBodyParticleEvalBase& eval_base )
		{
			CorrectDirections( head_body_particle );
			cv::Rect_<int> image_rect( eval_base.offset_x(), eval_base.offset_y(),
									   eval_base.image().cols, eval_base.image().rows );
			cv::Point2i particle_lu( head_body_particle.x(), head_body_particle.y() );
			cv::Point2i particle_rl( head_body_particle.x() + head_body_particle.s() - 1,
									 head_body_particle.y() + head_body_particle.s() - 1 );
			return ( image_rect.contains( particle_lu ) &&
					 image_rect.contains( particle_rl ) &&
					 head_body_particle.s() >= s_min_ && head_body_particle.s() <= s_max_ );
		}

	double Likelihood( const HeadBodyParticle& particle,
					   const HeadBodyParticleEvalBase& eval_base )
		{
			std::map<int, double> head_probs, body_probs;
			GetHeadProbs( eval_base, particle.x(), particle.y(), particle.s(), head_probs );
			GetBodyProbs( eval_base, body_probs );
			return GetDirectionProb( head_probs, body_probs, particle.dh(), particle.db() );
		}

	double GetBestParticle( const HeadBodyParticleEvalBase& eval_base,
							 HeadBodyParticle& best_particle)
		{
			double max_probability = -1;
			FindBestParticle find_best_particle( *this, eval_base, max_probability, best_particle );
#ifdef USING_TBB
			tbb::blocked_range3d<int, int, int> range(
				s_min_, s_max_ + 1, 100,
				eval_base.offset_x(), eval_base.offset_x() + eval_base.image().cols, 100,
				eval_base.offset_y(), eval_base.offset_y() + eval_base.image().rows, 100 );
			tbb::parallel_for( range, find_best_particle );
#else /* ifdef USING_TBB */
			find_best_particle();
#endif /* ifdef USING_TBB */
			return max_probability;
		}

private:
	/* typedef, struct */

	/* Methods */
	void CorrectDirection( int& direction, int direction_num )
		{
			if( direction < 0 || direction >= direction_num ) {
				while( direction < 0 ) {
					direction += direction_num;
				}
				while( direction >= direction_num ) {
					direction -= direction_num;
				}
				assert( direction < 0 || direction >= direction_num );
			}
		}

	void CorrectDirections( HeadBodyParticle& particle )
		{
			int head_direction = particle.dh();
			int body_direction = particle.db();
			CorrectDirection( head_direction, head_dir_num_ );
			CorrectDirection( body_direction, body_dir_num_ );
			particle.set_dh( head_direction );
			particle.set_db( body_direction );
		}

	void GetHeadProbs( const HeadBodyParticleEvalBase& eval_base,
					   int x, int y, int s, std::map<int, double>& head_probs )
		{
			cv::Rect_<int> roi_rect( x - eval_base.offset_x(), y - eval_base.offset_y(), s, s );
			cv::Mat roi_image = eval_base.image()( roi_rect );
			head_classifier_.PredictProbability( roi_image, head_probs );
		}

	void GetBodyProbs( const HeadBodyParticleEvalBase& eval_base,
					   std::map<int, double>& body_probs )
		{
			body_classifier_.PredictProbability( eval_base.image(), body_probs );
		}

	double GetDirectionDiffProb( int head_dir, int body_dir )
		{
			double head_angle = head_dir * M_PI * 0.25;
			double body_angle = body_dir * M_PI * 0.25;
			double angle_diff = body_angle - head_angle;
			while( angle_diff < -1 * M_PI ) {
				angle_diff += 2 * M_PI;
			}
			while( angle_diff > M_PI ) {
				angle_diff -= 2*M_PI;
			}
			return von_mises_pdf( angle_diff, angle_diff_mean_, angle_diff_beta_ );
		}

	double GetDirectionProb( std::map<int, double>& head_probs, std::map<int, double>& body_probs,
							 int head_dir, int body_dir )
		{
			double log_likelihood = log( head_probs[head_dir] ) + log( body_probs[body_dir] )
				+ log( GetDirectionDiffProb( head_dir, body_dir ) );
			if( log_likelihood < -10 ) {
				return 0.0;
			} else {
				return exp( log_likelihood );
			}
		}

	class FindBestParticle {
	public:
		FindBestParticle( HeadBodyParticleEvaluator& evaluator,
						  const HeadBodyParticleEvalBase& eval_base,
						  double& max_probability,
						  HeadBodyParticle& best_particle ):
			evaluator_(evaluator), eval_base_(eval_base),
			max_probability_(max_probability), best_particle_(best_particle) {}

#ifdef USING_TBB
		void operator()( const tbb::blocked_range3d<int, int, int>& range ) const
			{
				for( int s = range.pages().begin(); s != range.pages().end(); ++s ) {
					for( int x = range.rows().begin(); x != range.rows().end() &&
							 x < eval_base_.offset_x() + eval_base_.image().cols - s - 1; ++x ) {
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
					for( int x = eval_base_.offset_x(); x < eval_base_.offset_x() + eval_base_.image().cols - s; ++x ) {
						for( int y = eval_base_.offset_y(); y < eval_base_.offset_y() + eval_base_.image().rows - s; ++y ) {
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
		double FindBestDirections( int x, int y, int s,
								  const HeadBodyParticleEvalBase& eval_base,
								   int& head_dir, int& body_dir ) const
			{
				std::map<int, double> head_probs, body_probs;
				evaluator_.GetHeadProbs( eval_base, x, y, s, head_probs );
				evaluator_.GetBodyProbs( eval_base, body_probs );

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
		
		HeadBodyParticleEvaluator& evaluator_;
		const HeadBodyParticleEvalBase& eval_base_;
		double& max_probability_;
		HeadBodyParticle& best_particle_;
	};

	/* variable */
	int s_min_, s_max_, head_dir_num_, body_dir_num_;
	double angle_diff_mean_, angle_diff_beta_, angle_diff_weight_;
	tcpp::vision::ImageClassifierInterface& head_classifier_, body_classifier_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
