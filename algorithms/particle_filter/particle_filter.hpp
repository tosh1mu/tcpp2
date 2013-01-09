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

#include <cassert>
#include <cmath>
#include <vector>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <class PT>
class ParticleFilter {
public:
	ParticleFilter( int particle_num,
					ParticleGeneratorInterface& generator,
					ParticleEvaluatorInterface& evaluator ):
		particles_(), weights_(),
		generator_(generator), evaluator_(evaluator) {}
	
	void Initialize( const ParticleEvalBaseInterface& eval_base )
		{
			double initial_weight = 1.0 / static_cast<double>( particle_num_ );
			PT initial_particle;
			evaluator_.GetBestParticle( eval_base, initial_particle );

			std::vector<double> updated_weights;
			double weight_sum = 0.0;

			particles_.reserve( particle_num_ );
			for( int i = 0; i < particle_num_; ) {
				PT particle;
				generator_.Generate( particle );
				if( evaluator_.Validate( particle ) ) {
					double updated_weight = initial_weight * evaluator_.Likelihood( particle, eval_base );
					particles_.push_back( particle );
					updated_weights.push_back( updated_weight );
					weight_sum += updated_weight;
					++i;
				}
			}
			NormalizeWeights( weight_sum );
		}

	void UpdateParticles( const ParticleEvalBaseInterface& eval_base,
						  ParticleInterface& estimated_particle )
		{
			std::vector<int> particle_num_list;
			GetParticleNumList( particle_num_list );
			int new_particle_num = static_cast<int>( particle_num_list.size() );

			std::vector<ParticleInterface> prev_particles( particles_ );
			std::vector<double> prev_weights( weights_ );
			particles_.clear();
			weights_.clear();
			particles_.reserve( new_particle_num );
			weights_.reserve( new_particle_num );

			double weight_sum = 0.0;
			for( unsigned int i = 0, n = prev_particles.size(); i < n; ++i ) {
				for( int j = 0; j < particle_num_list[i]; ) {
					PT new_particle;
					generator_.Generate( prev_particles[i], new_particle );
					if( evaluator_.Validate( new_particle ) ) {
						particles_.push_back( new_particle );
						double likelihood = evaluator_.Likelihood( new_particle );
						double new_weight = prev_weights[i] * likelihood;
						weights_.push_back( new_weight );
						weight_sum += new_weight;
					}
				}
			}
			NormalizeWeights( weight_sum );
			generator_.GetWeightedMean( particles_, weights_, estimated_particle );
		}

private:
	/* Method */
	void NormalizeWeights( double weight_sum ) {
		for( std::vector<double>::iterator w_itr = weights_.begin();
			 w_itr != weights_.end(); ++w_itr ) {
			(*w_itr) /= weight_sum;
		}
	}

	void GetParticleNumList( std::vector<int> particle_num_list )
		{
			assert( particle_num_list.empty() );
			particle_num_list.reserve( particle_num_ );
			for( std::vector<double>::const_iterator w_itr = weights_.begin();
				 w_itr != weights_.end(); ++w_itr ) {
				int particle_num =
					static_cast<int>( floor( (*w_itr) * particle_num + 0.5 ) );
				particle_num_list.push_back( particle_num );
			}
		}

	/* Data member */
	int particle_num_;
	std::vector<ParticleInterface> particles_;
	std::vector<double> weights_;
	//
	ParticleGeneratorInterface& generator_;
	ParticleEvaluatorInterface& evaluator_;
}; /* ParticleFilter */

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_FILTER_HPP_ */
