/**
 * @file head_pose_particle.hpp
 * @brief Particle of human head pose (x, y, s, t)
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_HPP_
#define TCPP_HEAD_POSE_PARTICLE_HPP_

#include "particle_interface.hpp"

/**
 * @namespace tcpp
 */
namespace tcpp {

class HeadPoseParticle: public ParticleInterface {
public:
	/**
	 * @brief Default constructor
	 */
	HeadPoseParticle(): x_(0), y_(0), s_(0), d_(0) {}
	/**
	 * @brief Constructor
	 */
	HeadPoseParticle( int x, int y, int s, int d ):
		x_(x), y_(y), s_(s), d_(d) {}

	int x() const { return x_; }
	int y() const { return y_; }
	int s() const { return s_; }
	int d() const { return d_; }

	void set_x( int x ) { x_ = x; }
	void set_y( int y ) { y_ = y; }
	void set_s( int s ) { s_ = s; }
	void set_d( int d ) { d_ = d; }

private:
	/* Particle parameters */
	int x_, y_; //!< Position of the left-upper point of head ROI
	int s_; //!< Scale of head ROI
	int d_; //!< Direction label of the head
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_HPP_ */

