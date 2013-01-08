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

#define USING_TBB
#ifdef USING_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range3d.h>
#endif /* USING_TBB */

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
		tcpp::vision::ImageClassifierInterface& classifier
	):
		constraint_( x_min, x_max, y_min, y_max, s_min, s_max, d_min, d_max ),
		classifier_( classifier )
		{}

	/* method */
	bool Validate( HeadPoseParticle& head_pose_particle )
		{
			if( head_pose_particle.x() >= constraint_.x_min
				&& head_pose_particle.x() + head_pose_particle.s() <= constraint_.x_max
				&& head_pose_particle.y() >= constraint_.y_min
				&& head_pose_particle.y() + head_pose_particle.s() <= constraint_.y_max )
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

	double Likelihood( const HeadPoseParticle& particle,
					   const HeadPoseParticleEvalBase& eval_base )
		{
			cv::Rect_<int> roi_rect( particle.x(), particle.y(), particle.s(), particle.s() );
			cv::Mat roi_image = eval_base.image()( roi_rect );
			std::map<int, double> probabilities;
			classifier_.PredictProbability( roi_image, probabilities );
			return probabilities[particle.d()];
		}

	void GetBestParticle( const HeadPoseParticleEvalBase& eval_base,
							 HeadPoseParticle& best_particle)
		{
			FindBestParticle find_best_particle( *this, eval_base, best_particle );
#ifdef USING_TBB
			tbb::blocked_range3d<int, int, int> range( constraint_.x_min, constraint_.x_max, 100,
													   constraint_.y_min, constraint_.y_max, 100,
													   constraint_.s_min, constraint_.s_max, 100 );
			tbb::parallel_for( range, find_best_particle );
#else
			find_best_particle();
#endif /* USING_TBB */
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

	/* Methods */
	class FindBestParticle {
	public:
		FindBestParticle( HeadPoseParticleEvaluator& evaluator,
						  const HeadPoseParticleEvalBase& eval_base,
						  HeadPoseParticle& best_particle ):
			evaluator_(evaluator), eval_base_(eval_base),
			max_probability_(0), best_particle_(best_particle) {}

#ifdef USING_TBB
		void operator()( const tbb::blocked_range3d<int, int, int>& range )
			{
				for( int s = range.pages().begin(); s != range.pages().end(); ++s ) {
					for( int x = range.rows().begin();
						 x != range.rows().end() && x <= evaluator_.constraint_.x_max - s; ++x ) {
						for( int y = range.cols().begin();
							 y != range.cols().end() && y <= evaluator_.constraint_.y_max - s; ++y ) {
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

		void operator()()
			{
				for( int s = evaluator_.constraint_.s_min; s <= evaluator_.constraint_.s_max; ++s ) {
					for( int x = evaluator_.constraint_.x_min; x <= evaluator_.constraint_.x_max - s; ++x ) {
						for( int y = evaluator_.constraint_.y_min;
							 y <= evaluator_.constraint_.y_max - s; ++y ) {
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
								  int& best_direction )
			{
				cv::Rect_<int> roi_rect( x, y, s, s );
				cv::Mat roi_image = eval_base.image()( roi_rect );
				std::map<int, double> probabilities;
				evaluator_.classifier_.PredictProbability( roi_image, probabilities );

				best_direction = -1;
				double max_prob = 0.0;
				for( typename std::map<int, double>::const_iterator prob_itr = probabilities.begin();
					 prob_itr != probabilities.end();
					 ++prob_itr )
					{
						int label = prob_itr->first;
						double prob = prob_itr->second;
						if( prob > max_prob ) {
							best_direction = label;
							max_prob = prob;
						}
					}
				return max_prob;
			}
		
		HeadPoseParticleEvaluator& evaluator_;
		const HeadPoseParticleEvalBase& eval_base_;
		double max_probability_;
		HeadPoseParticle& best_particle_;
	};

	/* variable */
	Constraint constraint_;
	tcpp::vision::ImageClassifierInterface& classifier_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
