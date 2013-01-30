/**
 * @file head_body_particle.hpp
 * @brief Particle of human head and body pose (x, y, s, dh, db)
 * @author Toshimitsu Takahashi
 * @date 2013/1/30
 *
 */

#ifndef TCPP_HEAD_BODY_PARTICLE_HPP_
#define TCPP_HEAD_BODY_PARTICLE_HPP_

#include "tcpp2/algorithms/particle_filter/particle_interface.hpp"
#include <cmath>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HeadBodyParticle: public tcpp::ParticleInterface {
public:
	/**
	 * @brief Default constructor
	 */
	HeadBodyParticle(): x_(0), y_(0), s_(0), dh_(0), db_(0) {}

	/**
	 * @brief Constructor
	 */
	template <typename T1, typename T2, typename T3, typename T4, typename T5>
	HeadBodyParticle( T1 x, T2 y, T3 s, T4 dh, T5 db ):
		x_( static_cast<int>( floor(x + 0.5) ) ),
		y_( static_cast<int>( floor(y + 0.5) ) ),
		s_( static_cast<int>( floor(s + 0.5) ) ),
		dh_( static_cast<int>( floor(dh + 0.5) ) ),
		db_( static_cast<int>( floor(db + 0.5) ) ) {}

	/**
	 * @brief Copy constructor
	 */
	HeadBodyParticle( const HeadBodyParticle& particle ):
		x_(particle.x_), y_(particle.y_), s_(particle.s_), dh_(particle.dh_), db_(particle.db_) {}

	/**
	 * @brief
	 */
	HeadBodyParticle& Copy( const HeadBodyParticle& particle )
		{
			if( this == &particle ){
				return *this;
			} else {
				x_ = particle.x_;
				y_ = particle.y_;
				s_ = particle.s_;
				dh_ = particle.dh_;
				db_ = particle.db_;
				return *this;
			}
		}

	HeadBodyParticle& operator=( const HeadBodyParticle& particle )
		{
			return Copy( particle );
		}

		/**
	 * @brief Size getter
	 */
	int Size() const { return 5; }

	/* Getters */
	int x() const { return x_; }
	int y() const { return y_; }
	int s() const { return s_; }
	int dh() const { return dh_; }
	int db() const { return db_; }

	/* Setters */
	template <typename T1>
	void set_x( T1 x ) { x_ = static_cast<int>( floor(static_cast<double>(x) + 0.5) ); }
	template <typename T1>
	void set_y( T1 y ) { y_ = static_cast<int>( floor(static_cast<double>(y) + 0.5) ); }
	template <typename T1>
	void set_s( T1 s ) { s_ = static_cast<int>( floor(static_cast<double>(s) + 0.5) ); }
	template <typename T1>
	void set_dh( T1 dh ) { dh_ = static_cast<int>( floor(static_cast<double>(dh) + 0.5) ); }
	template <typename T1>
	void set_db( T1 db ) { db_ = static_cast<int>( floor(static_cast<double>(db) + 0.5) ); }

private:
	/* parameters */
	int x_, y_;
	int s_;
	int dh_;
	int db_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_BODY_PARTICLE_HPP_ */
