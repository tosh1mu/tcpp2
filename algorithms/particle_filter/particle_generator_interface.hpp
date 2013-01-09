/**
 * @file particle_generator_interface.hpp
 * @brief Interface of particle generators
 * @author Toshimitsu Takahashi
 * @date 2013/1/6
 * @version 0.0.1
 *
 */

#ifndef PARTICLE_GENERATOR_INTERFACE_HPP_
#define PARTICLE_GENERATOR_INTERFACE_HPP_

#include "particle_interface.hpp"
#include <vector>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <class PC>
class ParticleGeneratorInterface {
public:
	virtual ~ParticleGeneratorInterface() {}
	virtual void Generate(
		const PC& src_particle,
		PC& dst_particle
	) = 0;
	virtual void GetWeightedMean( const std::vector<PC>& particles,
								  const std::vector<double>& weights,
								  PC& mean_particle ) = 0;
};

} /* namespace tcpp */

#endif /* PARTICLE_GENERATOR_INTERFACE_HPP_ */





