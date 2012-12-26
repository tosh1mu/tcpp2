/**
 * @file feature_extractor.hpp
 * @brief <BRIEF EXPLANATION>
 * @author takahashi
 * @date 2012/12/19
 * @version 0.0.1
 */

#ifndef TCPP_FEATURE_EXTRACTOR_HPP_
#define TCPP_FEATURE_EXTRACTOR_HPP_

#include "../../core/core.hpp"
#include <opencv2/core/core.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace ip
 */
namespace ip {

/**
 * @class FeatureExtractor
 * @brief Interface class of feature extractors
 */
template <typename T>
class FeatureExtractor {
public:
	/**
	 * @brief Default constructor
	 */
	FeatureExtractor(): dimension_(-1) {}
	/**
	 * @brief Constructor
	 * @param dimension
	 */
	FeatureExtractor( size_t dimension ): dimension_(dimension) {}
	/**
	 * @brief Destructor
	 */
	virtual ~FeatureExtractor() {}

	/**
	 * @brief Accessor
	 * @return Dimension of feature vector
	 */
	size_t dimension() const { return dimension_; }

	/**
	 * @brief Interface to unify extracting function
	 * @param image
	 * @param feature
	 */
	virtual void Extract( const cv::Mat& image, std::vector<T>& features ) {}
protected:
	size_t dimension_; //!< dimension of feature vector
};

} /* namespace ip */
} /* namespace tcpp */


#endif /* TCPP_FEATURE_EXTRACTOR_HPP_ */
