/**
 * @file macros.hpp
 * @brief Common macros in tcpp2
 * @author Toshimitsu Takahashi
 * @date 2012/12/22
 *
 */

#ifndef TCPP2_MACROS_HPP_
#define TCPP2_MACROS_HPP_

#include <iostream>
#include <string>

/* info strings */
#define FUNC_NAME std::string(__func__)
#define FILE_NAME std::string(__FILE__)

/* info streams */
#define PLACE_STREAM __func__ << "() at line " << __LINE__ << " of " << __FILE__
#define ERROR_STREAM(MSG) "***ERROR***" << MSG << " [" << PLACE_STREAM << "]"
#define WARNING_STREAM(MSG) "***WARNING***" << MSG << " [" << PLACE_STREAM << "]"
#define CHECK_STREAM "***CHECK***" << " [" << PLACE_STREAM << "]"

/* print infos */
#define PRINT_ERROR(MSG) std::cerr << ERROR_STREAM(MSG) << std::endl
#define PRINT_WARNING(MSG) std::cerr << WARNING_STREAM(MSG) << std::endl
#define PRINT_CHECK std::cout << CHECK_STREAM << std::endl

/* streams of value */
#define VAL_STREAM(VAL) #VAL << " = " << VAL
#define VAL_STREAM2(VAL, DESC) DESC << "(" << #VAL << ") = " << VAL

/* print value */
#define PRINT_VAL(VAL) std::cout << VAL_STREAM(VAL) << std::endl
#define PRINT_VAL2(VAL, DESC) std::cout << VAL_STREAM2(VAL, DESC) << std::endl
#define PRINT_STDVEC(VEC) std::cout << #VEC << " = {" << VEC[0];	\
	for( size_t i = 1; i < VEC.size(); ++i ) {						\
		std::cout << ", " << VEC[i]; }								\
	std::cout << "}" << std::endl

#endif /* TCPP2_MACROS_HPP_ */

