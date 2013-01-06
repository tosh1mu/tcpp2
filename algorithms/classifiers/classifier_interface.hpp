/**
 * @file classifier_interface.hpp
 * @brief Definition of interface class for classifiers
 * @author Toshimitsu Takahashi
 * @data 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_CLASSIFIER_INTERFACE_HPP_
#define TCPP_CLASSIFIER_INTERFACE_HPP_

#include <string>
#include <vector>
#include <map>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T>
class ClassifierInterface {
public:
	virtual ~ClassifierInterface() {}
	virtual int Save( const std::string& path ) = 0;
	virtual int Load( const std::string& path ) = 0;
	virtual int GetClassNum() = 0;
	virtual int GetClassLabels( std::vector<int>& labels ) = 0;
	virtual int GetDimension() = 0;
	virtual int Predict( const std::vector<T>& x ) = 0;
	virtual int PredictProb( const std::vector<T>& x, std::map<int, double>& prob ) = 0;
}; /* ClassifierInterface */

} /* namespace tcpp */

#endif /* TCPP_CLASSIFIER_INTERFACE_HPP_ */
