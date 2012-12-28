/**
 * @file bkg_model_interface.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_BKG_MODEL_INTERFACE_HPP_
#define TCPP_BKG_MODEL_INTERFACE_HPP_

#include <string>
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
 * @brief Background model interface class
 */
class BkgModelInterface {
public:
	virtual ~BkgModelInterface() = 0;
	virtual const cv::Size2i& size() const = 0;
	virtual int learn_num() const = 0;
	virtual void Update( const cv::Mat& bkg_image ) = 0;
	virtual void Subtract( const cv::Mat& image, cv::Mat& mask ) = 0;
	virtual void Subtract( const cv::Mat& partial_image, const cv::Point2i& left_upper, cv::Mat& mask ) = 0;
	virtual void Process( const cv::Mat& image, cv::Mat& mask ) = 0;
	virtual int Save( const std::string& file_path ) const = 0;
	virtual int Load( const std::string& file_path ) = 0;
};

BkgModelInterface::~BkgModelInterface() {}

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_BKG_MODEL_INTERFACE_HPP_ */
