/*
 * stochastic.hpp
 *
 *  Created on: 2012/10/24
 *      Author: takahashi
 */

#ifndef TCPP_STOCHASTIC_HPP_
#define TCPP_STOCHASTIC_HPP_

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace tcpp {

template <typename T>
double GetExponentialProbabilityDensity(const T &x, const double &lambda) {
	double xx = static_cast<double>(x);
	return lambda * exp(-lambda*xx);
}

template <typename T>
double GetExponentialProbabilityDistribution(const T &x, const double &lambda) {
	double xx = static_cast<double>(x);
	return 1 - exp(-lambda*xx);
}

template <typename T>
double GetNormalProbabilityDensity(const T &x, const double &mean, const double &standard_deviation) {
	double variance = pow(standard_deviation,2);
	double xx = static_cast<double>(x);
	return exp(-pow(xx-mean,2)/(2*variance))/sqrt(2*M_PI*variance);
}

template <typename T, int dimension>
double GetMultinomialNormalProbabilityDensity(const Eigen::Matrix<T, dimension, 1>& x, const Eigen::Matrix<T, dimension, 1>& mean, const Eigen::Matrix<double, dimension, dimension>& covariance_matrix) {
	Eigen::Matrix<double, dimension, 1> xx = x.template cast<double>();
	Eigen::Matrix<double, dimension, 1> m = mean.template cast<double>();
	double probability = exp(-0.5*((xx-m).dot(covariance_matrix.inverse()*(xx-m))))/(pow(2*M_PI, static_cast<double>(dimension)/2)*sqrt(covariance_matrix.determinant()));
	return probability;
}

template <typename T>
T Get2DNormalProbabilityDensityDiscrete(const Eigen::Vector2i relative_vector, const cv::Size2i kernel_size, const double& sigma_x, const double& sigma_y, int normalize = 1) {
	int x = relative_vector.coeff(0);
	int y = relative_vector.coeff(1);
	if(abs(relative_vector.coeff(0)) > (kernel_size.width-1)/2 || abs(relative_vector.coeff(1)) > (kernel_size.height-1)/2)
		return 0.0;
	Eigen::ArrayXXd kernel = Eigen::ArrayXXd::Zero(kernel_size.height, kernel_size.width);
	double sum = 0.0;
	int center_row = (kernel_size.height-1)/2;
	int center_col = (kernel_size.width-1)/2;
	for(int kernel_row=0 ; kernel_row<kernel_size.height ; ++kernel_row) {
		for(int kernel_col=0 ; kernel_col<kernel_size.width ; ++kernel_col) {
			int xx = kernel_col - center_col;
			int yy = kernel_row - center_row;
			double probability = exp( -0.5* (pow(xx/sigma_x, 2) + pow(yy/sigma_y, 2)) ) / (sqrt(2*M_PI)*sigma_x*sigma_y);
			sum += probability;
			kernel(kernel_row, kernel_col) = probability;
		}
	}
	double density = kernel.coeff(y+center_row, x+center_col);
	if(normalize == 0) {
		density += 0.0;
	}else if(normalize == -1) {
		density /= kernel(center_row, center_row) * 0.90;
	}else{
		density /= sum;
	}
	return static_cast<T>(density);
}

} /* namespace tcpp */

#endif /* TCPP_STOCHASTIC_HPP_ */
