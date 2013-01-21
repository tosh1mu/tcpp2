/**
 * @file von_mises_rand.hpp
 * @brief Random number generator class using Python
 * @author takahashi
 * @date 2013/01/20
 * @version 0.0.1
 * @note require -I/usr/include/python2.7, -lboost_python-mt-py27, -lpython2.7
 */

#ifndef TCPP_VON_MISES_RAND_HPP_
#define TCPP_VON_MISES_RAND_HPP_

#define BOOST_PYTHON_STATIC_LIB

#include <python2.7/Python.h>
#include <boost/python.hpp>

namespace tcpp {

namespace boopy = boost::python;

class VonMisesRand {
public:
	VonMisesRand( double mean, double beta ): mean_(mean), beta_(beta) {
		Py_Initialize();
		global_ns_ = boopy::import("__main__").attr("__dict__");
		exec( "import random \n"
			"random.seed() \n"
			"def von_mises_rand( mean, beta ): \n"
			"	return random.vonmisesvariate( mean, beta ) \n", global_ns_, global_ns_ );
		von_mises_rand_ = global_ns_["von_mises_rand"];
	}
	~VonMisesRand() { Py_Finalize(); }
	double operator()() {
		boopy::object rand_val = von_mises_rand_( mean_, beta_ );
		return boopy::extract<double>( rand_val );
	}
private:
	boopy::object global_ns_, von_mises_rand_;
	double mean_, beta_;
};

} /* namespace tcpp */

#endif /* TCPP_VON_MISES_RAND_HPP_ */
