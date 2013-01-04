/**
 * @file svm.hpp
 * @brief Wrapper class for SVM of LIBSVM
 * @author Toshimitsu Takahashi
 * @data 2013/1/3
 * @version 0.0.1
 *
 */

#ifndef TCPP_SVM_HPP_
#define TCPP_SVM_HPP_

#include "classifier_interface.hpp"

#include <tcpp2/core/macros.hpp>
#include <tcpp2/core/string_numeric_convert.hpp>

#include <cassert>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>
#include <libsvm/svm.h>

/**
 * @namespace tcpp
 */
namespace tcpp {

template <typename T>
class SVM: public ClassifierInterface<T> {
public:
	SVM( const std::string& libsvm_model_file, const std::string& libsvm_scale_file ):
		libsvm_model_file_(libsvm_scale_file), scale_info_(libsvm_scale_file)
		{
			if( (libsvm_model_ = svm_load_model( libsvm_model_file.c_str() ) ) == 0 ) {
				PRINT_ERROR( "Could not load LIBSVM model from " + libsvm_model_file );
				abort();
			}
		}

	~SVM() {
		svm_free_and_destroy_model( &libsvm_model_ );
	}

	int Save( const std::string& path ) {
		return 1;
	}

	int Load( const std::string& path ) {
		return 1;
	}

	int GetClassNum() {
		return libsvm_model_->nr_class;
	}

	int GetClassLabels( std::vector<int>& labels ) {
		assert( labels.empty() );
		int class_num = libsvm_model_->nr_class;
		labels.reserve( class_num );
		for( int i = 0; i < class_num; ++i ) {
			labels.push_back( libsvm_model_->label[i] );
		}
		return class_num;
	}

	int GetDimension() {
		return scale_info_.dimension();
	}

	int Predict( const std::vector<T>& x ) {
		int dimension = scale_info_.dimension();
		assert( static_cast<int>( x.size() ) == dimension );
		std::vector<double> scaled_x;
		scale_info_.Scale( x, scaled_x );
		struct svm_node* nodes;
		nodes = (struct svm_node *) malloc( ( dimension + 1 ) * sizeof(struct svm_node) );
		GetSVMNodes( scaled_x, nodes );
		double predict_label = svm_predict( libsvm_model_, nodes );
		free( nodes );
		return static_cast<int>( predict_label );
	}

	int PredictProb( const std::vector<T>& x, std::map<int, double>& prob ) {
		int dimension = scale_info_.dimension();
		assert( static_cast<int>( x.size() ) == dimension );
		assert( prob.empty() );

		if( (libsvm_model_->param.svm_type == C_SVC || libsvm_model_->param.svm_type == NU_SVC)
			&& libsvm_model_->probA!=NULL && libsvm_model_->probB!=NULL ) {
			std::vector<double> scaled_x;
			scale_info_.Scale( x, scaled_x );
			struct svm_node* nodes;
			nodes = (struct svm_node *) malloc( ( dimension + 1 ) * sizeof(struct svm_node) );
			GetSVMNodes( scaled_x, nodes );
			int class_num = libsvm_model_->nr_class;
			double probs[class_num];
			double predict_label = svm_predict_probability( libsvm_model_, nodes, &probs );
			std::vector<int> class_labels;
			GetClassLabels( class_labels );
			for( int i = 0; i < class_num; ++i ) {
				prob[ class_labels[i] ] = probs[i];
			}
			return static_cast<int>( predict_label );
		} else {
			return Predict( x );
		}
	}

private:
	/* Inner Class */
	class ScaleInfo {
	public:
		ScaleInfo( const std::string& libsvm_scale_file ) {
			if( Load(libsvm_scale_file) == 1 ) {
				PRINT_ERROR("Could not read scaling info from " + libsvm_scale_file + "." );
				abort();
			}
		}

		void Scale( const std::vector<T>& x, std::vector<double>& scaled_x ) {
			assert( scaled_x.empty() );
			scaled_x.reserve( x.size() );
			for( size_t i = 0; i < x.size(); ++i ) {
				assert( dim_mins_.count(i+1) && dim_maxs_.count(i+1) );
				double val = static_cast<double>( x[i] );
				double scaled_val;
				if( val < dim_mins_[i+1] ) {
					val = dim_mins_[i+1];
				} else if( val > dim_maxs_[i+1] ) {
					val = dim_maxs_[i+1];
				}
				scaled_val = ( ( max_ - min_ ) / ( dim_maxs_[i+1] - dim_mins_[i+1] + 1E-10 ) )
					* ( val - dim_mins_[i+1] ) + min_;
				assert( !isnan(scaled_val) );
				scaled_x.push_back( scaled_val );
			}
		}

		int dimension() { return dimension_; }

	private:
		int Load( const std::string& libsvm_scale_file ) {
			std::ifstream ifs( libsvm_scale_file.c_str() );
			if( !ifs ) {
				return 1;
			} else {
				std::string line_buf;
				int line_count = 0;
				while( ifs && getline(ifs, line_buf) ) {
					if( line_count < 1 ) {
						++line_count;
						continue;
					} else {
						std::vector<std::string> split_line;
						boost::split( split_line, line_buf, boost::is_any_of(" ") );
						if( line_count == 1 ) {
							assert( split_line.size() == 2 );
							min_ = tcpp::numeric<double>( split_line[0] );
							max_ = tcpp::numeric<double>( split_line[1] );
						} else {
							assert( split_line.size() == 3 );
							int index = tcpp::numeric<int>( split_line[0] );
							double dim_min = tcpp::numeric<int>( split_line[1] );
							double dim_max = tcpp::numeric<int>( split_line[2] );;
							assert( index > 0 );
							assert( dim_min <= dim_max );
							dim_mins_[index] = dim_min;
							dim_maxs_[index] = dim_max;
						}
						++line_count;
					}
				}
				ifs.close();
				dimension_ = line_count - 2;
				return 0;
			}
		}

		double min_;
		double max_;
		std::map<int, double> dim_mins_;
		std::map<int, double> dim_maxs_;
		int dimension_;
	};

	/* Methods */
	void GetSVMNodes( const std::vector<T>& x, struct svm_node* nodes ) {
		for( size_t i = 0; i < x.size(); ++i ) {
			nodes[i].index = static_cast<int>(i) + 1;
			nodes[i].value = static_cast<double>(x[i]);
		}
		nodes[x.size()].index = -1;
	}

	std::string libsvm_model_file_;
	ScaleInfo scale_info_;
	struct svm_model* libsvm_model_;
};

} /* namespace tcpp */

#endif /* TCPP_SVM_HPP_ */









