/**
 * @file head_pose_particle_generator.hpp
 * @brief Generator of HeadPoseParticle
 * @author Toshimitsu Takahashi
 * @date 2013/1/6
 * @version 0.0.1
 *
 */

#ifndef TCPP_HEAD_POSE_PARTICLE_GENERATOR_HPP_
#define TCPP_HEAD_POSE_PARTICLE_GENERATOR_HPP_

#include "tcpp2/algorithms/particle_filter/particle_generator_interface.hpp"
#include "head_pose_particle.hpp"

#include <complex>
#include <stdexcept>
#include <tcpp2/core/rand_num_maker.hpp>

#ifdef USING_TBB
#include <tbb/queuing_mutex.h>
#endif /* USING_TBB */

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HeadPoseParticleGenerator:
		public tcpp::ParticleGeneratorInterface<HeadPoseParticle> {
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

	void Generate( const HeadPoseParticle& src_hp_particle,
				   HeadPoseParticle& dst_hp_particle )
		{
#ifdef USING_TBB
			tbb::queuing_mutex::scoped_lock lock( mutex );
#endif /* USING_TBB */
			dst_hp_particle.set_x<double>( static_cast<double>( src_hp_particle.x() ) + rand_x_() );
			dst_hp_particle.set_y<double>( static_cast<double>( src_hp_particle.y() ) + rand_y_() );
			dst_hp_particle.set_s<double>( static_cast<double>( src_hp_particle.s() ) + rand_s_() );
			dst_hp_particle.set_d<double>( static_cast<double>( src_hp_particle.d() ) + rand_d_() );
		}

	void GetWeightedMean( const ParticleContainer& particles,
						  const WeightContainer& weights,
						  HeadPoseParticle& mean_particle )
		{
			assert( particles.size() == weights.size() );
			int n = static_cast<int>( particles.size() );
			double sum_x = 0.0;
			double sum_y = 0.0;
			double sum_s = 0.0;
			double sum_weight = 0.0;
			std::complex<double> vec_d(0, 0);
			for( int i = 0; i < n; ++i ) {
				sum_x += static_cast<double>( particles[i].x() ) * weights[i];
				sum_y += static_cast<double>( particles[i].y() ) * weights[i];
				sum_s += static_cast<double>( particles[i].s() ) * weights[i];
				double phase = static_cast<double>( particles[i].d() ) * 2 * M_PI / 8.0;
				vec_d += std::complex<double>( weights[i]*std::cos(phase), weights[i]*std::sin(phase) );
				sum_weight += weights[i];
			}
			double denominator = 1.0 / sum_weight;
			mean_particle.set_x<double>( sum_x * denominator );
			mean_particle.set_y<double>( sum_y * denominator );
			mean_particle.set_s<double>( sum_s * denominator );
			if( std::norm(vec_d) <= 1e-10 ) {
				throw std::runtime_error( "Failed to calculate direction average." );
			}
			mean_particle.set_d<double>( std::arg(vec_d) * 8.0 / 2.0 / M_PI );
		}

private:
	/* Data member */
	// RandNumMaker
	RandNumMaker<boost::normal_distribution<> > rand_x_, rand_y_, rand_s_, rand_d_;
#ifdef USING_TBB
	tbb::queuing_mutex mutex;
#endif /* USING_TBB */
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HEAD_POSE_PARTICLE_GENERATOR_HPP_ */
