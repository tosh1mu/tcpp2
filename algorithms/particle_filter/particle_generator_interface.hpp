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

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleGeneratorInterface {
public:
	virtual ~ParticleInterface() {}
	virtual void Generate(
		const ParticleInterface& src_particle,
		ParticleInterface& dst_particle
	) = 0;
};

} /* namespace tcpp */

#endif /* PARTICLE_GENERATOR_INTERFACE_HPP_ */
