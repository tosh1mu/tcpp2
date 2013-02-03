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

#ifdef USING_TBB
#include <tbb/concurrent_vector.h>
#endif /* USING_TBB */

/**
 * @namespace tcpp
 */
namespace tcpp {

template <class PC>
class ParticleGeneratorInterface {
public:
#ifdef USING_TBB
	typedef tbb::concurrent_vector<PC> ParticleContainer;
	typedef tbb::concurrent_vector<double> WeightContainer;
#else
	typedef std::vector<PC> ParticleContainer;
	typedef std::vector<double> WeightContainer;
#endif

	virtual ~ParticleGeneratorInterface() {}
	virtual void Generate(
		const PC& src_particle,
		PC& dst_particle
	) = 0;
	virtual void GetWeightedMean( const ParticleContainer& particles,
								  const WeightContainer& weights,
								  PC& mean_particle ) = 0;
};

} /* namespace tcpp */

#endif /* PARTICLE_GENERATOR_INTERFACE_HPP_ */





