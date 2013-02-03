/**
 * @file head_body_particle_evaluator_m.hpp
 * @brief Evaluator of head body particle (with model)
 * @author Toshimitsu Takahashi
 * @date 2013/2/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_BODY_PARTICLE_EVALUATOR_M_HPP_
#define TCPP_HEAD_BODY_PARTICLE_EVALUATOR_M_HPP_

#include "tcpp2/algorithms/particle_filter/particle_evaluator_interface.hpp"
#include "head_body_particle.hpp"
#include "head_body_particle_eval_base_m.hpp"

#include "tcpp2/vision/image_classifiers/image_classifier_interface.hpp"

#include "tcpp2/core/macros.hpp"
#include "tcpp2/math/geometry_funcs.hpp"
#include "tcpp2/math/probability.hpp"
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

class HeadBodyParticleEvaluatorM:
		public tcpp::ParticleEvaluatorInterface<HeadBodyParticle, HeadBodyParticleEvalBaseM> {
public:
	/* constructor, copy, destructor */
	HeadBodyParticleEvaluatorM( int s_min, int s_max, int head_dir_num, int body_dir_num,
								double head_prob_weight, double body_prob_weight,
								double angle_diff_mean, double angle_diff_beta, double angle_diff_weight,
								tcpp::vision::ImageClassifierInterface& head_classifier,
								tcpp::vision::ImageClassifierInterface& body_classifier,
								const tcpp::Discrete2dNormalParams& head_dist_params ):
		s_min_(s_min), s_max_(s_max), head_dir_num_(head_dir_num), body_dir_num_(body_dir_num),
		head_prob_weight_(head_prob_weight), body_prob_weight_(body_prob_weight),
		angle_diff_mean_(angle_diff_mean), angle_diff_beta_(angle_diff_beta), angle_diff_weight_(angle_diff_weight),
		head_classifier_(head_classifier), body_classifier_(body_classifier),
		head_dist_params_(head_dist_params)
		{
			double head_angle_unit = 2 * M_PI / static_cast<double>(head_dir_num);
			double tmp = 0.0;
			for( int i = 0; i < head_dir_num; ++i ) {
				tmp += von_mises_pdf( head_angle_unit * i, angle_diff_mean, angle_diff_beta );
			}
			assert( tmp > 1E-10 );
			von_mises_norm_ = 1.0 / tmp;
		}

	/* method */
	bool Validate( HeadBodyParticle& head_body_particle,
				   const HeadBodyParticleEvalBaseM& eval_base )
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
					   const HeadBodyParticleEvalBaseM& eval_base )
		{
			std::map<int, double> head_probs, body_probs;
			GetHeadProbs( eval_base, particle.x(), particle.y(), particle.s(), head_probs );
			GetBodyProbs( eval_base, body_probs );
			double log_likelihood = 0.0;
			double dir_prob = GetDirectionProb( head_probs, body_probs, particle.dh(), particle.db() );
			double model_prob = GetProbWithModel( eval_base, particle.x(), particle.y(), particle.s() );
			assert( !isnan(dir_prob) && !isnan(model_prob) );

			if( dir_prob > 1E-10 && model_prob > 1E-10 ) {
				log_likelihood = log( dir_prob ) + log( model_prob );
				if ( log_likelihood > -10 ) {
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

	double GetBestParticle( const HeadBodyParticleEvalBaseM& eval_base,
							 HeadBodyParticle& best_particle)
		{
			double max_probability = -1;
			FindBestParticle find_best_particle( *this, eval_base, max_probability, best_particle );
#ifdef USING_TBB
			tbb::blocked_range3d<int, int, int> range(
				s_min_, s_max_ + 1, 100,
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
	void CorrectDirection( int& direction, int direction_num )
		{
			if( direction < 0 || direction >= direction_num ) {
				while( direction < 0 ) {
					direction += direction_num;
				}
				while( direction >= direction_num ) {
					direction -= direction_num;
				}
			}
			assert( direction >= 0 && direction < direction_num );
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

	void GetHeadProbs( const HeadBodyParticleEvalBaseM& eval_base,
					   int x, int y, int s, std::map<int, double>& head_probs )
		{
			cv::Rect_<int> roi_rect( x - eval_base.offset_x(), y - eval_base.offset_y(), s, s );
			cv::Mat roi_image = eval_base.image()( roi_rect );
			head_classifier_.PredictProbability( roi_image, head_probs );
		}

	void GetBodyProbs( const HeadBodyParticleEvalBaseM& eval_base,
					   std::map<int, double>& body_probs )
		{
			body_classifier_.PredictProbability( eval_base.image(), body_probs );
		}

	double GetDirectionDiffProb( int head_dir, int body_dir )
		{
			double head_angle = static_cast<double>(head_dir) * M_PI * 0.25;
			double body_angle = static_cast<double>(body_dir) * M_PI * 0.25;
			double angle_diff = body_angle - head_angle;
			
			while( angle_diff < -1 * M_PI ) {
				angle_diff += 2 * M_PI;
			}
			while( angle_diff > M_PI ) {
				angle_diff -= 2 * M_PI;
			}
			double pdf = von_mises_pdf( angle_diff, angle_diff_mean_, angle_diff_beta_ );
			return pdf * von_mises_norm_;
		}

	double GetDirectionProb( std::map<int, double>& head_probs, std::map<int, double>& body_probs,
							 int head_dir, int body_dir )
		{
			double head_prob = head_probs[head_dir];
			double body_prob = body_probs[body_dir];
			double diff_prob = GetDirectionDiffProb( head_dir, body_dir );
			assert( !isnan(head_prob) && !isnan(body_prob) && !isnan(diff_prob) );
			double log_likelihood = 0.0;
			if( head_prob > 1E-20 ) {
				log_likelihood += head_prob_weight_ * log( head_prob );
			} else {
				return 0.0;
			}
			if( body_prob > 1E-20 ) {
				log_likelihood += body_prob_weight_ * log( body_prob );
			} else {
				return 0.0;
			}
			if( diff_prob > 1E-20 ) {
				log_likelihood += angle_diff_weight_ * log( diff_prob );
			} else {
				return 0.0;
			}
			
			if( log_likelihood > -10 ) {
				double likelihood = exp( log_likelihood );
				assert( !isnan( likelihood ) );
				return likelihood;
			} else {
				return 0.0;
			}
		}

	double GetProbWithModel( const HeadBodyParticleEvalBaseM& eval_base,
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
		FindBestParticle( HeadBodyParticleEvaluatorM& evaluator,
						  const HeadBodyParticleEvalBaseM& eval_base,
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
								 y < eval_base_.offset_y() + eval_base_.head_area_height() - s - 1; ++y ) {
							int best_head_dir, best_body_dir;
							double probability = FindBestDirections( x, y, s, eval_base_, best_head_dir, best_body_dir );
							if( probability > max_probability_ ) {
								HeadBodyParticle particle( x, y, s, best_head_dir, best_body_dir );
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
							int best_head_dir, best_body_dir;
							double probability = FindBestDirections( x, y, s, eval_base_, best_head_dir, best_body_dir );
							if( probability > max_probability_ ) {
								HeadBodyParticle particle( x, y, s, best_head_dir, best_body_dir );
								best_particle_ = particle;
								max_probability_ = probability;
							}
						}
					}
				}
			}

	private:
		double FindBestDirections( int x, int y, int s,
								  const HeadBodyParticleEvalBaseM& eval_base,
								   int& best_head_dir, int& best_body_dir ) const
			{
				std::map<int, double> head_probs, body_probs;
				evaluator_.GetHeadProbs( eval_base, x, y, s, head_probs );
				evaluator_.GetBodyProbs( eval_base, body_probs );

				best_head_dir = -1;
				best_body_dir = -1;
				double max_prob = -1.0;
				for( typename std::map<int, double>::iterator head_itr = head_probs.begin();
					 head_itr != head_probs.end();
					 ++head_itr ) {
					int head_dir = head_itr->first;
					if( head_dir == -1 ) {
						continue;
					}
					for( typename std::map<int, double>::iterator body_itr = body_probs.begin();
						 body_itr != body_probs.end();
						 ++body_itr ) {
						int body_dir = body_itr->first;
						
						double prob = evaluator_.GetDirectionProb( head_probs, body_probs,
																   head_dir, body_dir );
						if( prob > max_prob ) {
							best_head_dir = head_dir;
							best_body_dir = body_dir;
							max_prob = prob;
						}
					}
				}
				assert( best_head_dir >= 0 && best_head_dir < evaluator_.head_dir_num_
						&& best_body_dir >= 0 && best_body_dir < evaluator_.body_dir_num_ );
				return max_prob;
			}
		
		HeadBodyParticleEvaluatorM& evaluator_;
		const HeadBodyParticleEvalBaseM& eval_base_;
		double& max_probability_;
		HeadBodyParticle& best_particle_;
	};

	/* variable */
	int s_min_, s_max_, head_dir_num_, body_dir_num_;
	double head_prob_weight_, body_prob_weight_;
	double angle_diff_mean_, angle_diff_beta_, angle_diff_weight_;
	double von_mises_norm_;
	tcpp::vision::ImageClassifierInterface& head_classifier_;
	tcpp::vision::ImageClassifierInterface& body_classifier_;
	const tcpp::Discrete2dNormalParams& head_dist_params_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_M_HPP_ */
