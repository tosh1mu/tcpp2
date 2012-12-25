/*
 * clustering.hpp
 *
 *  Created on: 2012/10/24
 *      Author: takahashi
 *
 *  Require: -lopencv_flann
 */

#ifndef TCPP_CLUSTERING_HPP_
#define TCPP_CLUSTERING_HPP_

#include "../../core/core.hpp"

#include <map>
#include <valarray>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>

namespace tcpp {
namespace prml {

template <typename T, unsigned int dimension>
double CalcBIC(const cv::Mat& cv_mat, const cv::Mat& centers, const cv::Mat_<int>& cluster_labels) {
	int data_num = cv_mat.rows;
	int cluster_num = centers.rows;

	int parameter_num = 1 + dimension * cluster_num + 1;

	// 各クラスタのデータ数のカウントと，varianceの最尤推定
	std::vector<int> data_num_of_each_cluster(cluster_num, 0);
	double estimated_variance = 0.0;
	for(int i=0 ; i<data_num ; ++i) {
		cv::Vec<float, dimension> data = cv_mat.at<cv::Vec<float, dimension> >(i, 0);
		int cluster = cluster_labels.at<int>(i);
		cv::Vec<float, dimension> center = centers.at<cv::Vec<float, dimension> >(cluster, 0);
		data_num_of_each_cluster[cluster] += 1;
		estimated_variance += pow(cv::norm(data-center), 2);
	}
	estimated_variance /= (data_num - cluster_num);

	// 各クラスタの尤度を最尤推定し，総和をとって全体の尤度を最尤推定する
	double maximum_log_likelihood = 0.0;
	for(int i=0 ; i<cluster_num ; ++i) {
		double estimated_maximum_log_likelihood = 0.0;
		estimated_maximum_log_likelihood -= data_num_of_each_cluster[i] * log(2*M_PI) / 2.0;
		estimated_maximum_log_likelihood -= data_num_of_each_cluster[i] * dimension * log(estimated_variance) / 2.0;
		estimated_maximum_log_likelihood -= (data_num_of_each_cluster[i] - cluster_num/(i+1)) / 2.0;
		estimated_maximum_log_likelihood += data_num_of_each_cluster[i] * log(data_num_of_each_cluster[i]);
		estimated_maximum_log_likelihood -= data_num_of_each_cluster[i] * log(data_num);
		maximum_log_likelihood += estimated_maximum_log_likelihood;
	}

	double bic = maximum_log_likelihood - (parameter_num * log(data_num))/2;
	return bic;
}

// 要素の型，データの次元数
template <typename T, size_t dimension>
void DivideClusters(const cv::Mat& src_mat, const cv::Mat_<int>& cluster_labels, std::map<int, cv::Mat>& clusters) {
	int data_num = src_mat.rows;
	assert( src_mat.channels() == dimension );
	std::map<int, std::vector<cv::Vec<T, dimension> > > cluster_data;
	for(int i=0 ; i<data_num ; ++i)
		cluster_data[ cluster_labels.at<int>(i) ].push_back( src_mat.at<cv::Vec<T, dimension> >(i) );

	if(!clusters.empty())
		clusters.clear();

	for( std::map<int, int>::iterator cluster_itr = cluster_data.begin() ; cluster_itr != cluster_data.end() ; ++cluster_itr ) {
		int label = (*cluster_itr).first;
		const std::vector<cv::Vec<T, dimension> >& data_list = (*cluster_itr).second;

		cv::Mat cluster_mat = cv::Mat::zeros(data_list.size(), 1, src_mat.type());

		for(int i=0 ; i<data_list.size() ; ++i)
			cluster_mat.at<cv::Vec<T, dimension> >(i) = data_list[i];

		clusters[label] = cluster_mat.clone();
	}
}

template <typename T, size_t dimension>
void DivideClusters(const cv::Mat& src_mat, const cv::Mat_<int>& cluster_labels, std::map<int, cv::Mat>& clusters, std::map<int, std::vector<int> >& indexes) {
	int data_num = src_mat.rows;
	assert( dimension == src_mat.channels() );
	std::map<int, std::vector<cv::Vec<T, dimension> > > cluster_data;
	for(int i=0 ; i<data_num ; ++i) {
		cluster_data[ cluster_labels.at<int>(i) ].push_back( src_mat.at<cv::Vec<T, dimension> >(i) );
		indexes[ cluster_labels.at<int>(i) ].push_back(i);
	}

	if(!clusters.empty())
		clusters.clear();

	for( std::map<int, int>::iterator cluster_itr = cluster_data.begin() ; cluster_itr != cluster_data.end() ; ++cluster_itr ) {
		int label = (*cluster_itr).first;
		const std::vector<cv::Vec<T, dimension> >& data_list = (*cluster_itr).second;

		cv::Mat cluster_mat = cv::Mat::zeros(data_list.size(), 1, src_mat.type());

		for(int i=0 ; i<data_list.size() ; ++i)
			cluster_mat.at<cv::Vec<T, dimension> >(i) = data_list[i];

		clusters[label] = cluster_mat.clone();
	}
}

// 要素の型，データの次元数
template <typename T, size_t dimension>
void DivideClusters(const cv::Mat& src_mat, const cv::Mat_<int>& cluster_labels, std::vector<cv::Mat>& clusters) {
	int data_num = src_mat.rows;
	assert( src_mat.channels() == dimension );
	std::map<int, std::vector<cv::Vec<T, dimension> > > cluster_data;
	for(int i=0 ; i<data_num ; ++i)
		cluster_data[ cluster_labels.at<int>(i) ].push_back( src_mat.at<cv::Vec<T, dimension> >(i) );

	if(!clusters.empty())
		clusters.clear();

	for( typename std::map<int, std::vector<cv::Vec<T, dimension> > >::iterator cluster_itr = cluster_data.begin() ; cluster_itr != cluster_data.end() ; ++cluster_itr ) {
//		int label = (*cluster_itr).first;
		const std::vector<cv::Vec<T, dimension> >& data_list = (*cluster_itr).second;

		cv::Mat cluster_mat = cv::Mat::zeros(data_list.size(), 1, src_mat.type());

		for(unsigned int i=0 ; i<data_list.size() ; ++i)
			cluster_mat.at<cv::Vec<T, dimension> >(i) = data_list[i];

		clusters.push_back(cluster_mat.clone());
	}
}

// 要素の型，データの次元数
template <typename T, size_t dimension>
void DivideClusters(const cv::Mat& src_mat, const cv::Mat_<int>& cluster_labels, std::vector<cv::Mat>& clusters, std::vector<std::vector<int> >& indexes) {
	int data_num = src_mat.rows;
	assert( src_mat.channels() == dimension );
	std::map<int, std::vector<cv::Vec<T, dimension> > > cluster_data;
	std::map<int, std::vector<int> > cluster_indexes;
	for(int i=0 ; i<data_num ; ++i) {
		cluster_data[ cluster_labels.at<int>(i) ].push_back( src_mat.at<cv::Vec<T, dimension> >(i) );
		cluster_indexes[ cluster_labels.at<int>(i) ].push_back(i);
	}

	if(!clusters.empty())
		clusters.clear();

	for( typename std::map<int, std::vector<cv::Vec<T, dimension> > >::iterator cluster_itr = cluster_data.begin() ; cluster_itr != cluster_data.end() ; ++cluster_itr ) {
		int label = (*cluster_itr).first;
		const std::vector<cv::Vec<T, dimension> >& data_list = (*cluster_itr).second;

		cv::Mat cluster_mat = cv::Mat::zeros(data_list.size(), 1, src_mat.type());

		for(unsigned int i=0 ; i<data_list.size() ; ++i)
			cluster_mat.at<cv::Vec<T, dimension> >(i) = data_list[i];

		clusters.push_back(cluster_mat.clone());
		indexes.push_back(cluster_indexes[label]);
	}
}

// 要素の型，データの次元数
template <typename T, unsigned int dimension>
int LocalSearchClustering(const cv::Mat& cv_mat, const double& search_radius, cv::Mat& centers, cv::Mat_<int>& cluster_labels) {
	int cluster_num = 0;
	int total_num = cv_mat.rows;
	std::vector<cv::Vec<T, dimension> > current_vecs;
	std::valarray<int> check_list(1, total_num);
	std::vector<cv::Vec<T, dimension> > center_vecs;
	center_vecs.reserve(total_num);
	cluster_labels = cv::Mat_<int>::zeros(total_num, CV_32SC1);

	while(check_list.sum() > 0) {
		// まだラベル付けされていないデータのうちの一つを新しいクラスタの種とする．
		// 種となるデータのインデックスをsrc_indexとする．
		int src_index = -1;
		for(int i=0 ; i<total_num ; ++i) {
			if(check_list[i] == 1) {
				src_index = i;
				break;
			}
		}

		// 新しいクラスタをつくる
		++cluster_num;
		int label = cluster_num;
		if(!current_vecs.empty())
			current_vecs.clear();
		current_vecs.reserve(check_list.sum());
		current_vecs.push_back(cv_mat.at<cv::Vec<T, dimension> >(src_index));

		// 種データにラベル付をする
		cluster_labels.at<int>(src_index) = label;
		check_list[src_index] = 0;

		// 種データが最後の未ラベルデータの場合，無駄なforループを避ける
		if(check_list.sum() == 0)
			continue;

		// 新しいクラスタに追加するデータを探す
		while(1) {
			int added_num = 0;
			for(int i=0 ; i<total_num ; ++i) {
				// 既にラベル付けが終わっているデータは無視
				if(check_list[i] == 0)
					continue;
				// まだラベルが付いていないデータを取り出し，
				const cv::Vec<T, dimension>& vec = cv_mat.at<cv::Vec<T, dimension> >(i);
				// 現時点で新クラスタに追加されている点のいずれかとの距離がsearch_radius以下であれば，クラスタにデータを追加する
				for(unsigned int j=0 ; j<current_vecs.size() ; ++j) {
					double distance = cv::norm(vec - current_vecs[j]);
					if(distance <= search_radius) {
						current_vecs.push_back(vec);
						added_num += 1;
						cluster_labels.at<int>(i) = label;
						check_list[i] = 0;
						break;
					}
				}
			}

			// 未ラベルデータを一周スキャンして，新たにクラスタに追加されるデータが見つからなければ現在のクラスタの生成は終了
			if(added_num == 0)
				break;
		}

		// クラスタの生成が終了したら，クラスタの重心を求める
		cv::Vec<T, dimension> center = current_vecs[0];
		for(unsigned int i=1 ; i<current_vecs.size() ; ++i) {
			center += current_vecs[i];
		}
		center = static_cast<double>(1.0/current_vecs.size()) * center;
		center_vecs.push_back(center);
	}

	// std::vectorに格納されていたクラスタ重心を，cv::Matにコピーする
	if(dimension > 1) {
		std::vector<cv::Mat> mat_vector;
		for(unsigned int dim=0 ; dim<dimension ; ++dim) {
			cv::Mat dim_mat = cv::Mat::zeros(cluster_num, 1, CV_32F);
			mat_vector.push_back(dim_mat);
		}
		cv::merge(mat_vector, centers);
	}else{
		centers = cv::Mat::zeros(cluster_num, 1, CV_32F);
	}
	for(int i=0 ; i<cluster_num ; ++i)
		centers.at<cv::Vec<T, dimension> >(i) = center_vecs[i];

	return cluster_num;
}

// 要素の型，データの次元数
template <typename T, unsigned int dimension>
int XMeansClustering(const cv::Mat& cv_mat, cv::Mat& centers, cv::Mat_<int>& cluster_labels) {
	std::vector<cv::Mat> cluster_stack, finished_clusters;
	std::vector<std::vector<int> > indexes_stack, finished_indexes;

	cluster_stack.push_back(cv_mat);
	std::vector<int> origin_indexes;
	origin_indexes.reserve(cv_mat.rows);
	for(int i=0 ; i<cv_mat.rows ; ++i)
		origin_indexes.push_back(i);
	indexes_stack.push_back(origin_indexes);

	int cluster_num = 1;

	while(!cluster_stack.empty()) {
		cv::Mat cluster = cluster_stack.back();
		cluster_stack.pop_back();

		std::vector<int> src_indexes = indexes_stack.back();
		indexes_stack.pop_back();

		cv::Mat centers;
		cv::Mat_<int> clusters(cluster.rows, 1, CV_32SC1);

		cv::kmeans(cluster, 1, clusters, cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 100, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);
		double origin_bic = CalcBIC<T, dimension>(cluster, centers, clusters);

		cv::kmeans(cluster, 2, clusters, cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 100, 1.0), 10, cv::KMEANS_PP_CENTERS, centers);
		double divide_bic = CalcBIC<T, dimension>(cluster, centers, clusters);

		if(divide_bic > origin_bic) {
			std::vector<cv::Mat> divided_clusters;
			std::vector<std::vector<int> > divided_indexes;
			DivideClusters<T, dimension>(cluster, clusters, divided_clusters, divided_indexes);
			for(int i=0 ; i<2 ; ++i) {
				cluster_stack.push_back(divided_clusters[i]);
				std::vector<int> indexes;
				indexes.reserve(divided_indexes[i].size());
				for(unsigned int j=0 ; j<divided_indexes[i].size() ; ++j) {
					indexes.push_back(src_indexes[divided_indexes[i][j]]);
				}
				indexes_stack.push_back(indexes);
			}
		}else{
			finished_clusters.push_back(cluster);
			finished_indexes.push_back(src_indexes);
			cluster_num += 1;
		}
	}
	return cluster_num;
}

} /* namespace prml */
} /* namespace tcpp */


#endif /* TCPP_CLUSTERING_HPP_ */
