/**
 * @file head_pose_particle_generator.hpp
 * @brief Generator to HeadPoseParticle
 * @author Toshimitsu Takahashi
 * @date 2013/1/6
 * @version 0.0.1
 *
 */

#ifndef HEAD_POSE_PARTICLE_GENERATOR_HPP_
#define HEAD_POSE_PARTICLE_GENERATOR_HPP_

#include "head_pose_particle.hpp"

#include <tcpp2/core/rand_num_maker.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleGenerator {
public:
	/* Constructor */
	HeadPoseParticle() {}

	void Generate(
		const HeadPoseParticle& src_hp_particle,
		HeadPoseParticle& dst_hp_particle
	)
		{
			
		}

private:
	/* Data member */
	
};

} /* namespace tcpp */

#endif /* HEAD_POSE_PARTICLE_GENERATOR_HPP_ */
