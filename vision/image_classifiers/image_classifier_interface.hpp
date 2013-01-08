/**
 * @file image_classifier_interface.cpp
 * @brief Definition of interface class of image classifiers
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_IMAGE_CLASSIFIER_INTERFACE_HPP_
#define TCPP_IMAGE_CLASSIFIER_INTERFACE_HPP_

#include <map>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class ImageClassifierInterface {
public:
	virtual ~ImageClassifierInterface() {}
	virtual void SetResizeSize( int resize_width, int resize_height ) = 0;
	virtual int Predict( const cv::Mat& image ) = 0;
	virtual int PredictProbability( const cv::Mat& image, std::map<int, double>& probabilities ) = 0;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_IMAGE_CLASSIFIER_INTERFACE_HPP_ */
