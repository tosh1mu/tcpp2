/**
 * @file particle_filter.hpp
 * @brief Particle Filter definition
 * @author Toshimitsu Takahashi
 * @date 2013/1/7
 * @version 0.0.1
 *
 */

#ifndef TCPP_PARTICLE_FILTER_HPP_
#define TCPP_PARTICLE_FILTER_HPP_

#include "particle_interface.hpp"
#include "particle_generator_interface.hpp"
#include "particle_eval_base_interface.hpp"
#include "particle_evaluator_interface.hpp"

#include <tcpp2/core/macros.hpp>

#include <cassert>
#include <cmath>
#include <vector>

#ifdef USING_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#endif /* USING_TBB */

/**
 * @namespace tcpp
 */
namespace tcpp {

template <class PC, class PEBC>
class ParticleFilter {
public:
	typedef typename ParticleGeneratorInterface<PC>::ParticleContainer ParticleContainer;
	typedef typename ParticleGeneratorInterface<PC>::WeightContainer WeightContainer;

	ParticleFilter( int particle_num,
					ParticleGeneratorInterface<PC>& generator,
					ParticleEvaluatorInterface<PC, PEBC>& evaluator ):
		particle_num_(particle_num), particles_(), weights_(),
		generator_(generator), evaluator_(evaluator)
		{
			particles_.reserve( particle_num );
			weights_.reserve( particle_num );
		}

	int particle_num() { return particle_num_; }
	std::vector<PC>& particles() { return particles_; }
	std::vector<double>& weights() { return weights_; }
	
	void Initialize( const PEBC& eval_base, PC& initial_particle )
		{
			double weight_sum;
			Init init( *this, eval_base, initial_particle, weight_sum );
#ifdef USING_TBB
			tbb::blocked_range<int> range( 0, particle_num_, 100 );
			tbb::parallel_for( range, init );
#else /* USING_TBB */
			init();
#endif /* USING_TBB */
			NormalizeWeights( weight_sum );
		}

	void UpdateParticles( const PEBC& eval_base, PC& estimated_particle )
		{
			double weight_sum;
			Update update( *this, eval_base, weight_sum );
#ifdef USING_TBB
			tbb::blocked_range<int> range( 0, static_cast<int>( update.prev_particles_.size() ), 100 );
			tbb::parallel_for( range, update );
			//update();
#else /* USING_TBB */
			update();
#endif /* USING_TBB */
			NormalizeWeights( weight_sum );
			generator_.GetWeightedMean( particles_, weights_, estimated_particle );
			assert( evaluator_.Validate( estimated_particle, eval_base ) );
		}

private:
	/* Method */
	class Init {
	public:
		Init( ParticleFilter& filter, const PEBC& eval_base, PC& initial_particle, double& weight_sum ):
			filter_(filter), eval_base_(eval_base), initial_particle_(initial_particle),
			initial_weight_( 1.0 / static_cast<double>( filter.particle_num_ ) ),
			weight_sum_(weight_sum)
			{
				filter_.evaluator_.GetBestParticle( eval_base_, initial_particle_ );
				weight_sum_ = 0.0;
			}

#ifdef USING_TBB
		void operator()( const tbb::blocked_range<int>& range ) const
			{
				for( int i = range.begin(); i != range.end(); ++i ) {
					PC particle;
					filter_.generator_.Generate( initial_particle_, particle );
					while( !filter_.evaluator_.Validate( particle, eval_base_ ) ) {
						filter_.generator_.Generate( initial_particle_, particle );
					}
					double updated_weight = filter_.evaluator_.Likelihood( particle, eval_base_ );
					filter_.particles_.push_back( particle );
					filter_.weights_.push_back( updated_weight );
					weight_sum_ += updated_weight;
				}
			}
#endif /* USING_TBB */

		void operator()() const
			{
				for( int i = 0; i < filter_.particle_num_; ++i ) {
					PC particle;
					filter_.generator_.Generate( initial_particle_, particle );
					while( !filter_.evaluator_.Validate( particle, eval_base_ ) ) {
						filter_.generator_.Generate( initial_particle_, particle );
					}
					double updated_weight = filter_.evaluator_.Likelihood( particle, eval_base_ );
					filter_.particles_.push_back( particle );
					filter_.weights_.push_back( updated_weight );
					weight_sum_ += updated_weight;
				}
			}

		ParticleFilter& filter_;
		const PEBC& eval_base_;
		PC& initial_particle_;
		double initial_weight_;
		double& weight_sum_;
	};

	class Update {
	public:
		Update( ParticleFilter& filter, const PEBC& eval_base, double& weight_sum ):
			filter_(filter), eval_base_(eval_base),
			prev_particles_( filter.particles_ ), weight_sum_(weight_sum)
			{
				filter.GetParticleNumList( particle_num_list_ );
				filter.particles_.clear();
				filter.weights_.clear();
				filter.particles_.reserve( filter.particle_num_ * 2 );
				filter.weights_.reserve( filter.particle_num_ * 2 );
				weight_sum_ = 0.0;
			}

#ifdef USING_TBB
		void operator()( const tbb::blocked_range<int>& range ) const
			{
				for( int i = range.begin(); i != range.end(); ++i ) {
					for( int j = 0, n = particle_num_list_[i]; j < n; ++j ) {
						PC new_particle;
						filter_.generator_.Generate( prev_particles_[i], new_particle );
						while( !filter_.evaluator_.Validate( new_particle, eval_base_ ) ) {
							filter_.generator_.Generate( prev_particles_[i], new_particle );
						}
						filter_.particles_.push_back( new_particle );

						double likelihood = filter_.evaluator_.Likelihood( new_particle, eval_base_ );
						// double new_weight = prev_weights_[i] * likelihood;
						double new_weight = likelihood;
						filter_.weights_.push_back( new_weight );
						weight_sum_ += new_weight;
					}
				}
			}
#endif /* USING_TBB */

		void operator()() const
			{
				for( int i = 0, m = static_cast<int>( prev_particles_.size() ); i < m; ++i ) {
					for( int j = 0, n = particle_num_list_[i]; j < n; ++j ) {
						PC new_particle;
						filter_.generator_.Generate( prev_particles_[i], new_particle );
						while( !filter_.evaluator_.Validate( new_particle, eval_base_ ) ) {
							filter_.generator_.Generate( prev_particles_[i], new_particle );
						}
						filter_.particles_.push_back( new_particle );
						double likelihood = filter_.evaluator_.Likelihood( new_particle, eval_base_ );
						// double new_weight = prev_weights[i] * likelihood;
						double new_weight = likelihood;
						filter_.weights_.push_back( new_weight );
						weight_sum_ += new_weight;
					}
				}
			}

		ParticleFilter& filter_;
		const PEBC& eval_base_;
		std::vector<int> particle_num_list_;
		ParticleContainer prev_particles_;
		//std::vector<double> prev_weights_;
		double& weight_sum_;
	};

	void NormalizeWeights( double weight_sum )
		{
			assert( weight_sum > 0.0 );
			double normalize_constant = 1.0 / weight_sum;
			for( typename WeightContainer::iterator w_itr = weights_.begin(); w_itr != weights_.end(); ++w_itr ) {
				(*w_itr) *= normalize_constant;
			}
		}

	void GetParticleNumList( std::vector<int>& particle_num_list )
		{
			assert( particle_num_list.empty() );
			particle_num_list.reserve( particle_num_ );
			for( typename WeightContainer::const_iterator w_itr = weights_.begin(); w_itr != weights_.end(); ++w_itr ) {
				int particle_num = static_cast<int>( floor( (*w_itr) * particle_num_ + 0.5 ) );
				particle_num_list.push_back( particle_num );
			}
		}

	/* Data member */
	int particle_num_;
	ParticleContainer particles_;
	WeightContainer weights_;
	//
	ParticleGeneratorInterface<PC>& generator_;
	ParticleEvaluatorInterface<PC, PEBC>& evaluator_;
}; /* ParticleFilter */

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_FILTER_HPP_ */
