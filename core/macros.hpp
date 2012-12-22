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
#include <boost/timer.hpp>

/* info strings */
#define FUNC_NAME std::string(__func__)
#define FILE_NAME std::string(__FILE__)

/* info streams */
#define PLACE_STREAM __func__ << "() at line " << __LINE__ << " of " << __FILE__
#define ERROR_STREAM(MSG) "***ERROR***" << MSG << "[" << PLACE_STREAM << "]"
#define WARNING_STREAM(MSG) "***WARNING***" << MSG << "[" << PLACE_STREAM << "]"
#define CHECK_STREAM "***CHECK***" << "[" << PLACE_STREAM << "]"

/* print infos */
#define PRINT_ERROR(MSG) std::cout << ERROR_STREAM(MSG) << std:;endl
#define PRINT_WARNING(MSG) std::cout << WARNING_STREAM(MSG) << std::endl
#define PRINT_CHECK std::cout << CHECK_STREAM << std::endl

/* streams of value */
#define VAL_STREAM(VAL) #VAL << " = " << VAL
#define VAL_STREAM2(VAL, DESC) DESC << "(" << #VAL << ") = " << VAL

#endif /* TCPP2_MACROS_HPP_ */

