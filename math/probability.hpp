/**
 * @file probability.hpp
 * @brief Fuctions of probabilities
 * @author Toshimitsu Takahashi
 * @date 2013/2/2
 * @version 0.0.1
 *
 */

#ifndef TCPP_PROBABILITY_HPP_
#define TCPP_PROBABILITY_HPP_

#include <eigen3/Eigen/Core>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T, int dimension>
double MultiNormalPdf( const Eigen::Matrix<T, dimension, 1>& x, const Eigen::Matrix<T, dimension, 1>& mean, const Eigen::Matrix<double, dimension, dimension>& covariance_matrix ) {
	Eigen::Matrix<double, dimension, 1> xx = x.template cast<double>();
	Eigen::Matrix<double, dimension, 1> m = mean.template cast<double>();
	double probability = exp(-0.5*((xx-m).dot(covariance_matrix.inverse()*(xx-m))))/(pow(2*M_PI, static_cast<double>(dimension)/2)*sqrt(covariance_matrix.determinant()));
	return probability;
}

template <typename T>
T Discrete2dNormalPdf(const Eigen::Vector2i& relative_vector, int kernel_width, int kernel_height, double sigma_x, double sigma_y, int normalize = 1) {
	int x = relative_vector.coeff(0);
	int y = relative_vector.coeff(1);
	if( abs( relative_vector.coeff(0) ) > ( kernel_width - 1 ) / 2 || abs( relative_vector.coeff(1) ) > ( kernel_height - 1 ) / 2 ) {
		return 0.0;
	}
	Eigen::ArrayXXd kernel = Eigen::ArrayXXd::Zero( kernel_height, kernel_width );
	double sum = 0.0;
	int center_row = ( kernel_height - 1 ) / 2;
	int center_col = ( kernel_width - 1 ) / 2;
	for( int kernel_row = 0; kernel_row < kernel_height; ++kernel_row ) {
		for(int kernel_col = 0; kernel_col < kernel_width; ++kernel_col ) {
			double xx = static_cast<double>( kernel_col - center_col );
			double yy = static_cast<double>( kernel_row - center_row );
			double probability = exp( - 0.5 * ( pow( xx / sigma_x, 2 ) + pow( yy / sigma_y, 2 ) ) ) / ( sqrt( 2*M_PI ) * sigma_x * sigma_y );
			sum += probability;
			kernel( kernel_row, kernel_col ) = probability;
		}
	}
	double density = kernel.coeff( y + center_row, x + center_col );
	if( normalize == 0 ) {
		density += 0.0;
	} else if( normalize == -1 ) {
		density /= kernel(center_row, center_row) * 0.90;
	} else {
		density /= sum;
	}
	return static_cast<T>( density );
}

struct Discrete2dNormalParams {
	int kernel_width, kernel_height;
	double sigma_x, sigma_y;
	int normalize;
};

template<typename T>
T Discrete2dNormalPdf( const Eigen::Vector2i& relative_vector, const Discrete2dNormalParams& params ) {
	return Discrete2dNormalPdf<T>( relative_vector, params.kernel_width, params.kernel_height, params.sigma_x, params.sigma_y, params.normalize );
}

} /* namespace tcpp */

#endif /* TCPP_PROBABILITY_HPP_ */
