/**
 * @file rand_num_maker.hpp
 * @brief 
 * @author Toshimitsu Takahashi
 * @date 2012/12/26
 *
 */

#ifndef TCPP_RAND_NUM_MAKER_HPP_
#define TCPP_RAND_NUM_MAKER_HPP_

#include <boost/random.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Random number generator class
 * @note Distribusion: boost::uniform_01<> etc.
 */
template <class Distribution, class Generator = boost::mt19937>
class RandNumMaker {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 */
	RandNumMaker():
		generator_( static_cast<unsigned long>( time(0) ) ),
		random_( generator_, distribution_ )
		{}

	/**
	 * @brief Constructor
	 * @param[in] a1
	 */
	template<typename T1>
	explicit RandNumMaker( T1 a1 ):
		generator_( static_cast<unsigned long>( time(0) ) ),
		distribution_(a1),
		random_( generator_, distribution_ )
		{}

	/**
	 * @brief Constructor
	 * @param[in] min Minimum value
	 * @param[in] max Maximum value
	 */
	template<typename T1, typename T2>
	RandNumMaker( T1 min, T2 max ):
		generator_( static_cast<unsigned long>( time(0) ) ),
		distribution_( min, max ),
		random_( generator_, distribution_ )
		{}

	/********** Operators **********/
	/**
	 * @brief Generate randome numeric value
	 * @return Random numeric value
	 */
	typename Distribution::result_type operator()() { return random_(); }
	
private:
	Generator generator_;
	Distribution distribution_;
	boost::variate_generator<Generator, Distribution> random_;
};

} /* namespace tcpp */

#endif /* TCPP_RAND_NUM_MAKER_HPP_ */
