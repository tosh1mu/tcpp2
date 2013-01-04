/**
 * @file lbp_svm_tmp.hpp
 * @brief Temporary definition of LBP+SVM classifier
 * @author Toshimitsu Takahashi
 * @date 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_LBP_SVM_TMP_HPP_
#define TCPP_LBP_SVM_TMP_HPP_

#include <tcpp2/algorithms/classifiers/svm.hpp>
#include <tcpp2/vision/feature_extractors/lbp_extractor.hpp>

#include <opencv2/imgproc/imgproc.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class LBPSVMTmp {
public:
	LBPSVMTmp( int lbp_rows, int lbp_cols, const std::string& libsvm_model_file, const std::string& libsvm_scale_file ):
		lbp_extractor_( lbp_rows, lbp_cols ), svm_( libsvm_scale_file, libsvm_scale_file ),
		resize_size_(0, 0) {}

	void SetResizeSize( int resize_width, int resize_height ) {
		resize_size_ = cv::Size2i( resize_width, resize_height );
	}

	int PredictProb( const cv::Mat& image, std::map<int, double>& probs ) {
		cv::Mat resized_image;
		if( resize_size_.width > 0 && resize_size_.height > 0 ) {
			cv::resize( image, resized_image, resize_size_ );
		} else {
			resized_image = image.clone();
		}
		std::vector<double> lbp;
		lbp_extractor_.Extract( resized_image, lbp );
		int label = svm_.PredictProb( lbp, probs );
		return label;
	}

private:
	LBPExtractor<double> lbp_extractor_;
	tcpp::SVM<double> svm_;
	cv::Size2i resize_size_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_LBP_SVM_TMP_HPP_ */
