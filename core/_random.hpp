/**
 * @file random.hpp
 * @brief 
 *  Created on: 2012/10/05
 *      Author: takahashi
 */

#ifndef TCPP_RANDOM_HPP_
#define TCPP_RANDOM_HPP_

#include "convert/string_numeric_convert.hpp"

#include <set>
#include <boost/random.hpp>
#include <boost/array.hpp>

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
	 * @brief
	 */
	RandNumMaker():
		generator_( static_cast<unsigned long>( time(0) ) ), random_( generator_, distribution_ ) {}
	
	template<typename T1>
	explicit RandNumMaker( T1 a1 ):
		generator_( static_cast<unsigned long>( time(0) ) ), distribution_(a1), random_( generator_, distribution_ ) {}
	
	template<typename T1, typename T2>
	RandNumMaker( T1 a1, T2 a2 ):
		generator_( static_cast<unsigned long>( time(0) ) ), distribution_( a1, a2 ), random_( generator_, distribution_ ) {}

	typename Distribution::result_type operator()() { return random_(); }
	
private:
	Generator generator_;
	Distribution distribution_;
	boost::variate_generator<Generator, Distribution> random_;
};

	template<size_t N, typename T>
	void GetArray( boost::array<T, N>& array ) {
		for( size_t i = 0; i < N; ++i ) {
			array[i] = static_cast<T>( random_() );
		}
	}

template <size_t N>
int GetRandom01Array( unsigned int select_number, boost::array<int, N> &output_array) {
	RandNumMaker<boost::uniform_int<> > random_maker(0, N-1);
	std::set<int> selected_index;
	while(selected_index.size() < select_number) {
		int index = random_maker();
		if(selected_index.find(index) == selected_index.end())
			selected_index.insert(index);
	}
	for(unsigned int ind=0 ; ind<N ; ++ind) {
		if(selected_index.find(ind) != selected_index.end())
			output_array[ind] = 1;
		else
			output_array[ind] = 0;
	}
	return 1;
}

template <size_t N>
int GetRandom01Vector( const unsigned int &select_number, std::vector<int> &output_vector ) {
	size_t num;
	if( N < 0 && !output_vector.empty() ) {
		num = output_vector.size();
	} else {
		num = N;
		if( !output_vector.empty() )
			output_vector.clear();
		output_vector.resize( num, 0 );
	}
	assert( select_number <= num );

	RandNumMaker<boost::uniform_int<> > random_maker(0, num-1);
	std::set<int> selected_index;
	while( selected_index.size() < select_number ) {
		int index = random_maker();
		if( selected_index.find(index) == selected_index.end() )
			selected_index.insert(index);
	}
	for( unsigned int ind = 0; ind < num; ++ind ) {
		if( selected_index.find(ind) != selected_index.end() )
			output_vector[ind] = 1;
		else
			output_vector[ind] = 0;
	}
	return 1;
}

} /* namespace tcpp */

#endif /* TCPP_RANDOM_HPP_ */
