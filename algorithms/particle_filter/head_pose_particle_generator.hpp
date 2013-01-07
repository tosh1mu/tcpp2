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

#include "particle_generator_interface.hpp"
#include "head_pose_particle.hpp"

#include <tcpp2/core/rand_num_maker.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleGenerator: public ParticleGeneratorInterface {
public:
	/* Constructor */
	HeadPoseParticleGenerator( double sigma_x, double sigma_y, double sigma_s, double sigma_d ):
		rand_x_(0, sigma_x), rand_y_(0, sigma_y), rand_s_(0, sigma_s), rand_d_(0, sigma_d)
		{
			RandNumMaker<boost::uniform_int<> > seeder( 0, static_cast<int>( time(0) ) );
			rand_x_.Seed( seeder() );
			rand_y_.Seed( seeder() );
			rand_s_.Seed( seeder() );
			rand_d_.Seed( seeder() );
		}

	void Generate(
		const HeadPoseParticle& src_hp_particle,
		HeadPoseParticle& dst_hp_particle
	)
		{
			dst_hp_particle.set_x( src_hp_particle.x() + rand_x_() );
			dst_hp_particle.set_y( src_hp_particle.y() + rand_y_() );
			dst_hp_particle.set_s( src_hp_particle.s() + rand_s_() );
			dst_hp_particle.set_d( src_hp_particle.d() + rand_d_() );
		}

private:
	/* Data member */
	// RandNumMaker
	RandNumMaker<boost::normal_distribution<> > rand_x_, rand_y_, rand_s_, rand_d_;
};

} /* namespace tcpp */

#endif /* HEAD_POSE_PARTICLE_GENERATOR_HPP_ */
