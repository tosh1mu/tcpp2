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
#include "particle_evaluator_interface.hpp"

#include <vector>

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleFilter {
public:
	ParticleFilter( int particle_num ) {}
private:
	/* Data member */
	std::vector<ParticleInterface> particles_;
	std::vector<double> weights_;
	//
	ParticleGeneratorInterface* generator_;
	ParticleEvaluatorInterface* evaluator_;
}; /* ParticleFilter */

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_FILTER_HPP_ */
