/**
 * @file particle_evaluator_interface.hpp
 * @brief Interface of particle evaluators
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_PARTICLE_EVALUATOR_INTERFACE_HPP_
#define TCPP_PARTICLE_EVALUATOR_INTERFACE_HPP_

#include "particle_interface.hpp"
#include "particle_eval_base_interface.hpp"
#include <vector>

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleEvaluatorInterface {
public:
	virtual ~ParticleEvaluatorInterface() {}

	virtual bool Validate(
		const ParticleInterface& particle,
		const ParticleEvalBaseInterface& eval_base ) = 0;

	virtual double Likelihood(
		const ParticleInterface& particle,
		const ParticleEvalBaseInterface& eval_base ) = 0;

	/**
	 * @brief Get particle which has maximum likelihood
	 */
	virtual double GetBestParticle(
		const ParticleEvalBaseInterface& eval_base,
		ParticleInterface& best_particle ) = 0;
};

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_EVALUATOR_INTERFACE_HPP_ */



