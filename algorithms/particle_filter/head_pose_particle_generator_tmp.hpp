/**
 * @file head_pose_particle_generator_tmp.hpp
 * @brief Temporary definition of generator of head pose particle
 * @author Toshimitsu Takahashi
 * @data 2013/1/3
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_GENERATER_TMP_HPP_
#define TCPP_HEAD_POSE_PARTICLE_GENERATER_TMP_HPP_

#include "head_pose_particle.hpp"
#include <tcpp2/core/rand_num_maker.hpp>

#include <cmath>

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticleGeneratorTmp {
public:
	HeadPoseParticleGeneratorTmp( double sigma_x, double sigma_y, double sigma_s, double sigma_d ):
		rand_x_(0, sigma_x), rand_y_(0, sigma_y), rand_s_(0, sigma_s), rand_d_(0, sigma_d) {}

	void Generate( const HeadPoseParticle& core_particle, HeadPoseParticle& new_particle ) {
		int x_offset = static_cast<int>( std::floor( rand_x_() + 0.5 ) );
		int y_offset = static_cast<int>( std::floor( rand_y_() + 0.5 ) );
		int s_offset = static_cast<int>( std::floor( rand_s_() + 0.5 ) );
		int d_offset = static_cast<int>( std::floor( rand_d_() + 0.5 ) );
		new_particle.set_x( core_particle.x() + x_offset );
		new_particle.set_y( core_particle.y() + y_offset );
		new_particle.set_s( core_particle.s() + s_offset );
		new_particle.set_d( core_particle.d() + d_offset );
	}
private:
	RandNumMaker<boost::normal_distribution<> > rand_x_, rand_y_, rand_s_, rand_d_;
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_GENERATER_TMP_HPP_ */
