/**
 * @file geometry_funcs.hpp
 * @brief Definitions of geometry functions
 * @author takahashi
 * @date 2013/01/20
 * @version 0.0.1
 * 
 */

#ifndef TCPP_GEOMETRY_FUNCS_HPP_
#define TCPP_GEOMETRY_FUNCS_HPP_

#include <cmath>
#include <opencv2/core/core.hpp>

namespace tcpp {

double PointSegmentDistance( double x0, double y0, double x1, double y1, double pt_x, double pt_y )
{
	double dx = x1 - x0;
	double dy = y1 - y0;
	double a = dx * dx + dy * dy;
	if( a == 0.0 ) {
		return sqrt( pow(x0 - pt_x, 2) + pow(y0 - pt_y, 2) );
	} else {
		double b = dx * ( x0 - pt_x) + dy * ( y0 - pt_y );
		double t =  - (b / a);
		if( t < 0.0 )
			t = 0.0;
		if( t > 1.0 )
			t = 1.0;
		double x = t * dx + x0;
		double y = t * dy + y0;
		return sqrt( pow( x - pt_x, 2 ) + pow( y - pt_y, 2 ) );
	}
}

} /* namespace tcpp */

#endif /* TCPP_GEOMETRY_FUNCS_HPP_ */
