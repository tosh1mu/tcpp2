/**
 * @file lbp_svm_classifier.hpp
 * @brief LBP+SVM image classifier class
 * @author Toshimitsu Takahashi
 * @date 2013/1/5
 * @version 0.0.1
 *
 */

#ifndef TCPP_LBP_SVM_CLASSIFIER_HPP_
#define TCPP_LBP_SVM_CLASSIFIER_HPP_

#include "image_classifier_interface.hpp"

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

class LbpSvmClassifier: public ImageClassifierInterface {
public:
	LbpSvmClassifier( int lbp_rows, int lbp_cols, const std::string& libsvm_model_file, const std::string& libsvm_scale_file ):
		lbp_extractor_( lbp_rows, lbp_cols ), svm_( libsvm_model_file, libsvm_scale_file ),
		resize_size_(0, 0) {}

	void SetResizeSize( int resize_width, int resize_height ) {
		resize_size_ = cv::Size2i( resize_width, resize_height );
	}

	int Predict( const cv::Mat& image ) {
		std::vector<double> lbp;
		ExtractLbp( image, lbp );
		return svm_.Predict( lbp );
	}

	int PredictProbability( const cv::Mat& image, std::map<int, double>& probabilities ) {
		std::vector<double> lbp;
		ExtractLbp( image, lbp );
		int label = svm_.PredictProb( lbp, probabilities );
		return label;
	}

private:
	void ExtractLbp( const cv::Mat& image, std::vector<double>& lbp ) {
		cv::Mat resized_image;
		if( resize_size_.width > 0 && resize_size_.height > 0 ) {
			cv::resize( image, resized_image, resize_size_ );
		} else {
			resized_image = image.clone();
		}
		lbp_extractor_.Extract( resized_image, lbp );
	}

	LBPExtractor<double> lbp_extractor_;
	tcpp::SVM<double> svm_;
	cv::Size2i resize_size_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_LBP_SVM_CLASSIFIER_HPP_ */
