/**
 * @file feature_extractor_interface.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_FEATURE_EXTRACTOR_INTERFACE_HPP_
#define TCPP_FEATURE_EXTRACTOR_INTERFACE_HPP_

#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

/**
 * @brief Interface class of feature extractors
 */
template <typename T>
class FeatureExtractorInterface {
public:
	/**
	 * @brief Accessor
	 * @return Dimension of feature vector
	 */
	virtual size_t dimension() const = 0;

	/**
	 * @brief Interface to unify extracting function
	 * @param[in] image
	 * @param[out] feature
	 */
	virtual void Extract( const cv::Mat& image, std::vector<T>& features ) = 0;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_FEATURE_EXTRACTOR_INTERFACE_HPP_ */
