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
#include "head_pose_particle_eval_base.hpp"

#include <tcpp2/vision/image_classifiers/image_classifier_interface.hpp>

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

class HeadPoseParticleEvaluator:
		public ParticleEvaluatorInterface<HeadPoseParticle, HeadPoseParticleEvalBase> {
public:
	/* constructor, copy, destructor */
	HeadPoseParticleEvaluator( int s_min, int s_max, int d_min, int d_max,
			tcpp::vision::ImageClassifierInterface& classifier ):
		s_min_(s_min), s_max_(s_max), d_min_(d_min), d_max_(d_max),
		classifier_( classifier ) {}

	/* method */
	bool Validate( HeadPoseParticle& head_pose_particle,
				   const HeadPoseParticleEvalBase& eval_base )
		{
			CorrectDirection( head_pose_particle );
			cv::Rect_<int> image_rect( eval_base.offset_x(), eval_base.offset_y(),
									   eval_base.offset_x() + eval_base.image().cols - 1,
									   eval_base.offset_y() + eval_base.image().rows - 1 );
			cv::Point2i particle_lu( head_pose_particle.x(), head_pose_particle.y() );
			cv::Point2i particle_rl( head_pose_particle.x() + head_pose_particle.s() - 1,
									 head_pose_particle.y() + head_pose_particle.s() - 1 );
			return ( image_rect.contains( particle_lu ) &&
					 image_rect.contains( particle_rl ) &&
					 head_pose_particle.s() >= s_min_ && head_pose_particle.s() <= s_max_ &&
					 head_pose_particle.d() >= d_min_ && head_pose_particle.d() <= d_max_ );
		}

	double Likelihood( const HeadPoseParticle& particle,
					   const HeadPoseParticleEvalBase& eval_base )
		{
			std::map<int, double> probabilities;
			GetProbabilities( eval_base, particle.x(), particle.y(), particle.s(), probabilities );
			return probabilities[ particle.d() ];
		}

	double GetBestParticle( const HeadPoseParticleEvalBase& eval_base,
							 HeadPoseParticle& best_particle)
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
	void CorrectDirection( HeadPoseParticle& head_pose_particle )
		{
			int src_d = head_pose_particle.d();
			if( src_d < d_min_ || src_d > d_max_ ) {
				int d_cycle = d_max_ - d_min_ + 1;
				int d_correct = src_d;
				while( d_correct < d_min_ ) {
					d_correct += d_cycle;
				}
				while( d_correct > d_max_ ) {
					d_correct -= d_cycle;
				}
				assert( d_correct >= d_min_ && d_correct <= d_max_ );
				head_pose_particle.set_d( d_correct );
			}
		}

	void GetProbabilities( const HeadPoseParticleEvalBase& eval_base,
						   int x, int y, int s, std::map<int, double>& probabilities )
		{
			cv::Rect_<int> roi_rect( x - eval_base.offset_x(), y - eval_base.offset_y(), s, s );
			cv::Mat roi_image = eval_base.image()( roi_rect );
			classifier_.PredictProbability( roi_image, probabilities );
		}

	class FindBestParticle {
	public:
		FindBestParticle( HeadPoseParticleEvaluator& evaluator,
						  const HeadPoseParticleEvalBase& eval_base,
						  double& max_probability,
						  HeadPoseParticle& best_particle ):
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
		double FindBestDirection( int x, int y, int s,
								  const HeadPoseParticleEvalBase& eval_base,
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
		
		HeadPoseParticleEvaluator& evaluator_;
		const HeadPoseParticleEvalBase& eval_base_;
		double& max_probability_;
		HeadPoseParticle& best_particle_;
	};

	/* variable */
	int s_min_, s_max_, d_min_, d_max_;
	tcpp::vision::ImageClassifierInterface& classifier_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
