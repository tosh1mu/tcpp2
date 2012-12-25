/*
 * time.hpp
 *
 *  Created on: 2012/10/05
 *      Author: takahashi
 */

#ifndef TCPP_TIME_HPP_
#define TCPP_TIME_HPP_

#include <ctime>

namespace tcpp {

enum TimeStringFormat {
	kYYMMDD_HHMMSS,
	kYY_MM_DD_HH_MM_SS
};

inline std::string GetTimeStringFormat( const TimeStringFormat& format ) {
	switch(format) {
	case kYYMMDD_HHMMSS:
		return "%y%m%d_%H%M%S";
	case kYY_MM_DD_HH_MM_SS:
		return "%y-%m-%d_%H-%M-%S";
	default:
		return "%y%m%d_%H%M%S";
	}
}

inline std::string GetTimeString( TimeStringFormat F = kYY_MM_DD_HH_MM_SS ) {
	std::string format = GetTimeStringFormat(F);
	time_t timer = time(NULL);
	struct tm *date;
	char str[256];
	date = localtime( &timer );
	std::strftime( str, sizeof(str), format.c_str(), date );
	std::string time(str);
	return time;
}

} /* namespace tcpp */

#endif /* TCPP_TIME_HPP_ */
