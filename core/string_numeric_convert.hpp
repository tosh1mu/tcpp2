/**
 * @file string_numeric_convert.hpp
 * @brief Convert string <-> numeric
 * @author Toshimitsu Takahashi
 * @date 2012/12/22
 *
 */

#ifndef TCPP2_STRING_NUMERIC_CONVERT_HPP_
#define TCPP2_STRING_NUMERIC_CONVERT_HPP_

#include <cmath>
#include <string>
#include <sstream>
#include <typeinfo>

/**
 * @namespace tcpp
 */
namespace tcpp {

/**
 * @brief Get std::string of numeric(int, float, double) value
 * @param[in] numeric Numeric value(int, float, double)
 * @param[in] digit Number of digit(only valid when int value given)
 * @return std::string of numeric value
 */
template <typename T>
std::string string( T numeric, unsigned int digit = 0 ) {
	std::ostringstream final_ss, tmp_ss;

	if( numeric < 0 )
		final_ss << "-";

	T abs = std::abs( numeric );
	unsigned int input_digit;
	tmp_ss << abs;
	std::string abs_str = tmp_ss.str();
	input_digit = abs_str.size();

	if( input_digit < digit && typeid(T) == typeid(int) ) {
		for( unsigned int i=0; i<digit - input_digit; ++i )
			final_ss << "0";
	}

	final_ss << abs_str;
	return final_ss.str();
}

/**
 * @brief Get numeric value converted from std::string
 * @param[in] string std::string which expresses numeric value
 * @return numeric value (type T)
 */
template <typename T>
T numeric( std::string string ) {
	T numeric;
	std::istringstream iss;
	iss.str( string );
	iss >> numeric;
	return numeric;
}

} /* namespace tcpp */

#endif /* TCPP2_STRING_NUMERIC_CONVERT_HPP_ */
