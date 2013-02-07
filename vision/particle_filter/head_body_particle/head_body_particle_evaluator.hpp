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

#include "tcpp2/core/macros.hpp"
#include "tcpp2/core/algorithm.hpp"
#include <cassert>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
							   double head_prob_weight, double body_prob_weight,
							   double angle_diff_weight, double angle_diff_sigma,
							   tcpp::vision::ImageClassifierInterface& head_classifier,
							   tcpp::vision::ImageClassifierInterface& body_classifier
	):
		s_min_(s_min), s_max_(s_max), head_dir_num_(head_dir_num), body_dir_num_(body_dir_num),
		head_prob_weight_(head_prob_weight), body_prob_weight_(body_prob_weight),
		angle_diff_weight_(angle_diff_weight),
		head_classifier_(head_classifier), body_classifier_(body_classifier)
		{
			angle_diff_kernel_ = cv::getGaussianKernel( 9, angle_diff_sigma, CV_64F );
			double kernel_sum = 0.0;
			for( int i = 0; i < 9; ++i ) {
				if( i < 2 || i > 6 ) {
					angle_diff_kernel_.at<double>( i, 0 ) = 0.0;
				} else {
					kernel_sum += angle_diff_kernel_.at<double>( i, 0 );
				}
			}
			angle_diff_kernel_ /= kernel_sum;
		}

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
			// AdjustProbs( head_probs, body_probs, 0.5, 0.1 );
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

	void AdjustProbs( std::map<int, double>& head_probs, std::map<int, double>& body_probs,
					  double sigma, double max_hp_diff )
		{
			int max_body_label;
			tcpp::GetMaxVal<int, double>( body_probs, max_body_label );
			Gaussianize( body_probs, max_body_label, sigma );
			AdjustHeadProbs( head_probs, max_body_label, max_hp_diff );
		}

	void Gaussianize( std::map<int, double>& prob, int center_label, double sigma )
		{
			cv::Mat gauss_kernel = cv::getGaussianKernel( 9, sigma, CV_64F );
			for( std::map<int, double>::iterator itr = prob.begin(); itr != prob.end(); ++itr ) {
				int label = itr->first;
				if( label < 0 )
					continue;
				int distance = LabelDistance( center_label, label );
				itr->second = gauss_kernel.at<double>( 4 + distance, 0 );
			}
		}

	void AdjustHeadProbs( std::map<int, double>& head_probs, int max_body_label, double max_diff )
		{
			assert( max_diff < 1.0 );
			double max_prob = -0.1, min_prob = 1.1, sum_prob = 0.0;
			for( std::map<int, double>::iterator itr = head_probs.begin(); itr != head_probs.end(); ++itr ) {
				int label = itr->first;
				double prob = itr->second;
				if( LabelDistance( max_body_label, label ) < -2 || LabelDistance( max_body_label, label ) > 2 ) {
					itr->second = 0.0;
				} else {
					if( prob > max_prob ) {
						max_prob = prob;
					}
					if( prob < min_prob ) {
						min_prob = prob;
					}
					sum_prob += prob;
				}
			}
			double constant = ( ( max_prob - min_prob ) - ( max_diff * sum_prob ) ) / ( 5.0 * max_diff );
			if( constant < -min_prob ) {
				constant = 0.0;
			}
			for( std::map<int, double>::iterator itr = head_probs.begin(); itr != head_probs.end(); ++itr ) {
				int label = itr->first;
				if( LabelDistance( max_body_label, label ) < -2 || LabelDistance( max_body_label, label ) > 2 ) {
					continue;
				} else {
					double src_prob = itr->second;
					itr->second = ( src_prob + constant ) / ( sum_prob + 5.0 * constant );
				}
			}
		}

	int LabelDistance( int src_label, int dst_label )
		{
			int distance = dst_label - src_label;
			if( distance < -4 )
				distance += 8;
			if( distance > 4 )
				distance -= 8;
			assert( distance >= -4 && distance <= 4 );
			return distance;
		}

	double GetDirectionDiffProb( int head_dir, int body_dir )
		{
			int diff = LabelDistance( body_dir, head_dir );
			return angle_diff_kernel_.at<double>( diff + 4 );
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
								  const HeadBodyParticleEvalBase& eval_base,
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
		
		HeadBodyParticleEvaluator& evaluator_;
		const HeadBodyParticleEvalBase& eval_base_;
		double& max_probability_;
		HeadBodyParticle& best_particle_;
	};

	/* variable */
	int s_min_, s_max_, head_dir_num_, body_dir_num_;
	double head_prob_weight_, body_prob_weight_, angle_diff_weight_;
	cv::Mat angle_diff_kernel_;
	tcpp::vision::ImageClassifierInterface& head_classifier_;
	tcpp::vision::ImageClassifierInterface& body_classifier_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_EVALUATER_HPP_ */
