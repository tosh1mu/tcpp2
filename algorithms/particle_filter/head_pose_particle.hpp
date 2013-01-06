/**
 * @file head_pose_particle.hpp
 * @brief Particle of human head pose (x, y, s, d)
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
	template <typename T1, typename T2, typename T3, typename T4>
	HeadPoseParticle( T1 x, T2 y, T3 s, T4 d ):
		x_( static_cast<int>( floor(x + 0.5) ) ),
		y_( static_cast<int>( floor(y + 0.5) ) ),
		s_( static_cast<int>( floor(s + 0.5) ) ),
		d_( static_cast<int>( floor(d + 0.5) ) ) {}


	/**
	 * @brief Size getter
	 */
	int Size() const { return 4; }

	/* Getters */
	int x() const { return x_; }
	int y() const { return y_; }
	int s() const { return s_; }
	int d() const { return d_; }

	/* Setters */
	template <typename T1>
	void set_x( T1 x ) { x_ = static_cast<int>( floor(x + 0.5) ); }
	template <typename T1>
	void set_y( T1 y ) { y_ = static_cast<int>( floor(y + 0.5) ); }
	template <typename T1>
	void set_s( T1 s ) { s_ = static_cast<int>( floor(s + 0.5) ); }
	template <typename T1>
	void set_d( T1 d ) { d_ = static_cast<int>( floor(d + 0.5) ); }

private:
	/* Parameters */
	int x_, y_; //!< Position of the left-upper point of head ROI
	int s_; //!< Scale of head ROI
	int d_; //!< Direction label of the head
};

} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_HPP_ */

