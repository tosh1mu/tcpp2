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
	virtual ~ParticleGeneratorInterface() {}
	virtual void Generate(
		const PC& src_particle,
		PC& dst_particle
	) = 0;
#ifdef USING_TBB
	virtual void GetWeightedMean( const tbb::concurrent_vector<PC>& particles,
								  const tbb::concurrent_vector<double>& weights,
								  PC& mean_particle ) = 0;
#else /* USING_TBB */
	virtual void GetWeightedMean( const std::vector<PC>& particles,
								  const std::vector<double>& weights,
								  PC& mean_particle ) = 0;
#endif /* USING_TBB */
};

} /* namespace tcpp */

#endif /* PARTICLE_GENERATOR_INTERFACE_HPP_ */





