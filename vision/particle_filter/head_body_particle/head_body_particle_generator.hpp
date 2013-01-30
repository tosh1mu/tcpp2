/**
 * @file head_body_particle_generator.hpp
 * @brief Generator of HeadBodyParticle
 * @author Toshimitsu Takahashi
 * @date 2013/1/30
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_BODY_PARTICLE_GENERATOR_HPP_
#define TCPP_HEAD_BODY_PARTICLE_GENERATOR_HPP_

#include "tcpp2/algorithms/particle_filter/particle_generator_interface.hpp"
#include "head_body_particle.hpp"

#include <tcpp2/core/rand_num_maker.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HeadBodyParticleGenerator:
		public tcpp::ParticleGeneratorInterface<HeadBodyParticle> {
public:
	/* Constructor */
	HeadBodyParticleGenerator( double sigma_x, double sigma_y, double sigma_s, double sigma_dh, double sigma_db ):
		rand_x_(0, sigma_x), rand_y_(0, sigma_y), rand_s_(0, sigma_s), rand_dh_(0, sigma_dh), rand_db_(0, sigma_db)
		{
			RandNumMaker<boost::uniform_int<> > seeder( 0, static_cast<int>( time(0) ) );
			rand_x_.Seed( seeder() );
			rand_y_.Seed( seeder() );
			rand_s_.Seed( seeder() );
			rand_dh_.Seed( seeder() );
			rand_db_.Seed( seeder() );
		}

	void Generate( const HeadBodyParticle& src_hb_particle,
				   HeadBodyParticle& dst_hb_particle )
		{
			dst_hb_particle.set_x<double>( static_cast<double>( src_hb_particle.x() ) + rand_x_() );
			dst_hb_particle.set_y<double>( static_cast<double>( src_hb_particle.y() ) + rand_y_() );
			dst_hb_particle.set_s<double>( static_cast<double>( src_hb_particle.s() ) + rand_s_() );
			dst_hb_particle.set_dh<double>( static_cast<double>( src_hb_particle.dh() ) + rand_dh_() );
			dst_hb_particle.set_db<double>( static_cast<double>( src_hb_particle.db() ) + rand_db_() );
		}

	void GetWeightedMean( const std::vector<HeadBodyParticle>& particles,
						  const std::vector<double>& weights,
						  HeadBodyParticle& mean_particle )
		{
			assert( particles.size() == weights.size() );
			int n = static_cast<int>( particles.size() );
			double sum_x = 0.0;
			double sum_y = 0.0;
			double sum_s = 0.0;
			double sum_dh = 0.0;
			double sum_db = 0.0;
			double sum_weight = 0.0;
			for( int i = 0; i < n; ++i ) {
				sum_x += static_cast<double>( particles[i].x() ) * weights[i];
				sum_y += static_cast<double>( particles[i].y() ) * weights[i];
				sum_s += static_cast<double>( particles[i].s() ) * weights[i];
				sum_dh += static_cast<double>( particles[i].dh() ) * weights[i];
				sum_db += static_cast<double>( particles[i].db() ) * weights[i];
				sum_weight += weights[i];
			}
			double denominator = 1.0 / sum_weight;
			mean_particle.set_x<double>( sum_x * denominator );
			mean_particle.set_y<double>( sum_y * denominator );
			mean_particle.set_s<double>( sum_s * denominator );
			mean_particle.set_dh<double>( sum_dh * denominator );
			mean_particle.set_db<double>( sum_db * denominator );
		}

private:
	/* Data member */
	// RandNumMaker
	RandNumMaker<boost::normal_distribution<> > rand_x_, rand_y_, rand_s_, rand_dh_, rand_db_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_GENERATOR_HPP_ */
