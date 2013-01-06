/**
 * @file particle_interface.hpp
 * @brief Interface class of particle
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 *
 */

#ifndef TCPP_PARTICLE_INTERFACE_HPP_
#define TCPP_PARTICLE_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleInterface {
public:
	virtual ~ParticleInterface() {}
	virtual int Size() const = 0;
};

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_INTERFACE_HPP_ */
