/**
 * @file particle_evaluater_interface.hpp
 * @brief Definition of interface class which validates and evaluates particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_PARTICLE_EVALUATER_INTERFACE_HPP_
#define TCPP_PARTICLE_EVALUATER_INTERFACE_HPP_

#include "particle_interface.hpp"

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleEvaluaterInterface {
public:
	virtual ~ParticleEvaluaterInterface() = 0;
	virtual bool Validate( const ParticleInterface& particle ) = 0;
	virtual double Likelihood( const ParticleInterface& particle ) = 0;
};

ParticleEvaluaterInterface::~ParticleEvaluaterInterface() {}

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_EVALUATER_INTERFACE_HPP_ */
