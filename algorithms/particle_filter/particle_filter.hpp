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
			double total_weight = 0.0;

			particles_.reserve( particle_num_ );
			for( int i = 0; i < particle_num_; ) {
				PT particle;
				generator_.Generate( particle );
				if( evaluator_.Validate( particle ) ) {
					double updated_weight = initial_weight * evaluator_.Likelihood( particle, eval_base );
					particles_.push_back( particle );
					updated_weights.push_back( updated_weight );
					total_weight += updated_weight;
					++i;
				}
			}
		}

private:
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
