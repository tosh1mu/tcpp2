/**
 * @file particle_field_interface.hpp
 * @brief Interface class of field of particle used by particle evaluation
 * @author Toshitsu Takahashi
 * @date 2013/1/5
 * @version 0.0.1
 *
 */

#ifndef TCPP_PARTICLE_FIELD_INTERFACE_HPP_
#define TCPP_PARTICLE_FIELD_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleFieldInterface {
public:
	virtual ~ParticleFieldInterface() = 0;
};

ParticleFieldInterface::~ParticleFieldInterface() {}

} /* namespace tcpp */

#endif
