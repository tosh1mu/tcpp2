/**
 * @file hog_svm_classifier.hpp
 * @brief HOG+SVM image classifier class
 * @author Toshimitsu Takahashi
 * @date 2013/1/5
 * @version 0.0.1
 *
 */

#ifndef TCPP_HOG_SVM_CLASSIFIER_HPP_
#define TCPP_HOG_SVM_CLASSIFIER_HPP_

#include "image_classifier_interface.hpp"

#include <tcpp2/algorithms/classifiers/svm.hpp>
#include <tcpp2/vision/feature_extractors/hog_extractor.hpp>

#include <opencv2/imgproc/imgproc.hpp>

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class HogSvmClassifier: public ImageClassifierInterface {
public:
	HogSvmClassifier( int hog_cell_rows, int hog_cell_cols,
					  int hog_block_rows, int hog_block_cols,
					  int hog_grad_orient_num,
					  const std::string& libsvm_model_file,
					  const std::string& libsvm_scale_file ):
		hog_extractor_( hog_cell_rows, hog_cell_cols, hog_block_rows, hog_block_cols,
						hog_grad_orient_num ),
		svm_( libsvm_model_file, libsvm_scale_file ),
		resize_size_(0, 0){}

	void SetResizeSize( int resize_width, int resize_height )
		{
			resize_size_ = cv::Size2i( resize_width, resize_height );
		}

	int Predict( const cv::Mat& image )
		{
			std::vector<double> hog;
			ExtractHog( image, hog );
			return svm_.Predict( hog );
		}

	int PredictProbability( const cv::Mat& image, std::map<int, double>& probabilities )
		{
			std::vector<double> hog;
			ExtractHog( image, hog );
			int label = svm_.PredictProb( hog, probabilities );
			return label;
		}

private:
	void ExtractHog( const cv::Mat& image, std::vector<double>& hog )
		{
			cv::Mat resized_image;
			if( resize_size_.width > 0 && resize_size_.height > 0 ) {
				cv::resize( image, resized_image, resize_size_ );
			} else {
				resized_image = image.clone();
			}
			if( resized_image.channels() > 1 ) {
				cv::cvtColor( resized_image, resized_image, CV_BGR2GRAY );
			}
			hog_extractor_.Extract( resized_image, hog );
		}

	HOGExtractor<double> hog_extractor_;
	tcpp::SVM<double> svm_;
	cv::Size2i resize_size_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_HOG_SVM_CLASSIFIER_HPP_ */
