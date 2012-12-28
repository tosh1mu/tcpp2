/**
 * @file mean_bkg_model.hpp
 * @brief
 * @author Toshimitsu Takahashi
 * @date 2012/12/28
 *
 */

#ifndef TCPP_MEAN_BKG_MODEL_HPP_
#define TCPP_MEAN_BKG_MODEL_HPP_

#include "bkg_model_interface.hpp"

/**
 * @namespace tcpp
 */
namespace tcpp {
/**
 * @namespace vision
 */
namespace vision {

class MeanBkgModel: public BkgModelInterface {
public:
	/********** Constructors **********/
	/**
	 * @brief Constructor
	 */
	MeanBkgModel( int width, int height, double low_threshold, double high_threshold ):
		size_(width, height), learn_num_(0) {}
	/********** Methods **********/
	/**
	 * @brief Getter
	 */
	const cv::Size2i& size() const { return size_; }
	/**
	 * @brief Getter
	 */
	int learn_num() const { return learn_num_; }
	/**
	 * @brief Update bkg model
	 */
	void Update( const cv::Mat& bkg_image ) {
		assert( bkg_image.size() == size_ );
		if( learn_num_ == 0 ) {
			bkg_image.convertTo( mean_mat_, CV_64F );
			variance_mat_ = cv::Mat::zeros( mean_mat_.size(), CV_64F );
		} else {
			assert( bkg_image.size() == mean_mat_.size() && bkg_image.channels() == mean_mat_.channels() );
			cv::Mat _bkg_image;
			bkg_image.convertTo( _bkg_image, CV_64F );
			double alpha = 1.0 / ( 1.0 + static_cast<double>( learn_num_ ) );
			variance_mat_ = ( 1.0 - alpha ) * variance_mat_ + alpha * ( 1.0 - alpha )
				* ( _bkg_image - mean_mat_ ).mul( _bkg_image - mean_mat_ );
			mean_mat_ = ( 1.0 - alpha ) * mean_mat_ + alpha * _bkg_image;
		}
		++learn_num_;
	}
	/**
	 * @brief Background subtraction
	 */
	void Subtract( const cv::Mat& image, cv::Mat& mask ) {
		assert( image.size() == mean_mat_.size() && image.channels() == mean_mat_.channels() );
		GenerateModel( image.depth() );
		cv::inRange( image, low_mat_, high_mat_, mask );
		mask = 255 - mask;
	}
	/**
	 * @brief Partial background subtraction
	 */
	void Subtract( const cv::Mat& partial_image, const cv::Point2i& left_upper, cv::Mat& mask ) {
		assert( partial_image.channels() == mean_mat_.channels() );
		cv::Rect_<int> roi( left_upper.x, left_upper.y, partial_image.cols, partial_image.rows );
		cv::Rect_<int> image_rect( 0, 0, mean_mat_.cols, mean_mat_.rows );
		assert( image_rect.contains( roi.tl() ) && image_rect.contains( roi.br() ) );
		GenerateModel( partial_image.depth() );
		cv::inRange( partial_image, low_mat_(roi), high_mat_(roi), mask );
		mask = 255 - mask;
	}
	/**
	 * @brief Subtraction and updating
	 */
	void Process( const cv::Mat& image, cv::Mat& mask ) {
		Subtract( image, mask );
		Update( image );
	}
	/**
	 * @brief Serialize the model
	 */
	int Save( const std::string& file_path ) const {
		cv::FileStorage storage( file_path.c_str(), cv::FileStorage::WRITE );
		if( storage.isOpened() ) {
			cv::write( storage, "size", size_ );
			cv::write( storage, "learn_num", learn_num_ );
			cv::write( storage, "low_threshold", low_threshold_ );
			cv::write( storage, "high_threshold", high_threshold_ );
			cv::write( storage, "mean_mat", mean_mat_ );
			cv::write( storage, "variance_mat", variance_mat_ );
			return 0;
		} else {
			return 1;
		}
	}
	/**
	 * @brief Deserialize the model
	 */
	int Load( const std::string& file_path ) {
		cv::FileStorage storage( file_path.c_str(), cv::FileStorage::READ );
		if( storage.isOpened() ) {
			storage["size"] >> size_;
			learn_num_ = static_cast<int>( storage["learn_num"] );
			low_threshold_ = static_cast<double>( storage["low_threshold"] );
			high_threshold_ = static_cast<double>( storage["high_threshold"] );
			storage["mean_mat_"] >> mean_mat_;
			storage["variance_mat"] >> variance_mat_;
			return 0;
		} else {
			return 1;
		}
	}
private:
	/********** Methods **********/
	/**
	 * @brief Genarate model
	 */
	void GenerateModel( int cv_depth ) {
		cv::Mat std_dev_mat, _low_mat, _high_mat;
		cv::sqrt( variance_mat_, std_dev_mat );
		_low_mat = mean_mat_ - std_dev_mat * low_threshold_;
		_high_mat = high_mat_ = mean_mat_ + std_dev_mat * high_threshold_;
		_low_mat.convertTo( low_mat_, cv_depth );
		_high_mat.convertTo( high_mat_, cv_depth );
	}
	/********** Data member **********/
	cv::Size2i size_;
	int learn_num_;
	double low_threshold_, high_threshold_;
	cv::Mat mean_mat_, variance_mat_;
	cv::Mat low_mat_, high_mat_;
};

} /* namespace vision */
} /* namespace tcpp */

#endif /* TCPP_MEAN_BKG_MODEL_HPP_ */
