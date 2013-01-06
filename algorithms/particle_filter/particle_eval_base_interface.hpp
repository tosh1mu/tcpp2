/**
 * @file particle_eval_base_interface.hpp
 * @brief Interface of base classes for particle evaluation
 * @author Toshitsu Takahashi
 * @date 2013/1/6
 * @version 0.0.1
 *
 */

#ifndef TCPP_PARTICLE_EVAL_BASE_INTERFACE_HPP_
#define TCPP_PARTICLE_EVAL_BASE_INTERFACE_HPP_

/**
 * @namespace tcpp
 */
namespace tcpp {

class ParticleEvalBaseInterface {
public:
	virtual ~ParticleEvalBaseInterface() {}
	virtual bool IsSet() const = 0;
};

} /* namespace tcpp */

#endif /* TCPP_PARTICLE_EVAL_BASE_INTERFACE_HPP_ */
