/*
 * lapjv.h
 *
 *  Created on: 2012/09/27
 *      Author: takahashi
 */

#ifndef TCPP_LAPJV_HPP_
#define TCPP_LAPJV_HPP_

#include "../../core/core.hpp"

#include <list>
#include <vector>
#include <deque>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace tcpp {
namespace mp {

class LAPJV {
public:
	LAPJV(const Eigen::ArrayXXd &cost_mat): cost_mat_(cost_mat) {
		n_ = cost_mat_.rows();
		x_ = Eigen::ArrayXi::Zero(n_); // columns assigned to rows
		y_ = Eigen::ArrayXi::Zero(n_); // rows assigned to columns
		u_ = Eigen::ArrayXd::Zero(n_); // dual row variables
		v_ = Eigen::ArrayXd::Zero(n_); // dual column variables
	}

	virtual ~LAPJV() {}

	double Solve(const int &row_reduction_times) {
		ColumnReduction();
		std::vector<int> init_freeRows;
		ReductionTransfer(init_freeRows);
		std::vector<int> freeRows;
		AugmentingRowReduction(init_freeRows, freeRows);
		for(int time=1 ; time<row_reduction_times ; time++) {
			std::vector<int> tmp_freeRows(freeRows);
			freeRows.clear();
			AugmentingRowReduction(tmp_freeRows, freeRows);
		}
		Augmentation(freeRows);
		CheckAssignments();
		///
		double cost = 0.0;
		for(int row=1 ; row<=n_ ; row++) {
			int col = x_(row-1);
			u_(row-1) = cost_mat_(row-1, col-1) - v_(col-1);
			cost += u_(row-1) + v_(col-1);
		}
		return cost;
	}
private:
	static const double inf_ = 1E15;
	static const int epsilon_ = 1E-05;
	const Eigen::ArrayXXd &cost_mat_;
	int n_;
	Eigen::ArrayXi x_, y_;
	Eigen::ArrayXd u_, v_;
	////
	int CheckMat(const Eigen::ArrayXXd &cost_mat) {
		// コスト行列のサイズを確認
		if( cost_mat.cols() != n_ || cost_mat.rows() != n_)
			return 0;
		else
			return 1;
	}

	int CheckAssignments() {
		Eigen::ArrayXi col_check = y_;
		int unassigned = n_;
		int error = 0;
		for(int row=1 ; row<=n_ ; ++row) {
			int col = x_.coeff(row-1);
			if( col != 0 ) {
				assert( row == y_.coeff(col-1) );
				unassigned -= 1;
				col_check(col-1) = 0;
			}
		}
		assert( col_check.sum() == 0 );
		return error;
	}
	////
	int ColumnReduction() {
		// 各列，最小コストの行を割り当て
		// 重複割り当て要求のあった行が割り当てられた列（最も右側）については，行番号を負にすることでマーキング
		for(int col=n_ ; col>0 ; col--) {
			double minCost = cost_mat_(0, col-1);
			int minRow = 1;
			for(int row=2 ; row<=n_ ; row++) {
				if(cost_mat_(row-1, col-1) < minCost) {
					minCost = cost_mat_(row-1, col-1);
					minRow = row;
				}
			}
			v_(col-1) = minCost;
			if(x_(minRow-1) == 0) {
				x_(minRow-1) = col;
				y_(col-1) = minRow;
			}else{
				x_(minRow-1) = -abs(x_.coeff(minRow-1));
				y_(col-1) = 0;
			}
		}
		return 0;
	}

	int ReductionTransfer(std::vector<int> &free_rows) {
		// 1:unassigned rowの確認
		// 2:column_reductionにおいて後から同じ行の割り当てを要求されなかった列のみｖを計算
		// 3:freeRowsに結果を格納
		free_rows.reserve(n_);
		for(int row=1 ; row<=n_ ; row++) {
			if(x_(row-1) == 0) {
				free_rows.push_back(row);
			} else if(x_(row-1) < 0) {
				x_(row-1) = abs(x_.coeff(row-1));
			} else {
				int assigned_col = x_(row-1);
				double Min = inf_;
				for(int col=1 ; col<=n_ ; col++)
					if(col != assigned_col)
						if(cost_mat_(row-1,col-1) - v_(col-1) < Min)
							Min = cost_mat_(row-1,col-1) - v_(col-1);
				v_(assigned_col-1) -= Min;
			}
		}
		return 0;
	}

	int AugmentingRowReduction(const std::vector<int> &free_rows, std::vector<int> &new_free_rows) {
		// unassigned rowの配列を受け取り，より適切な割り当てになるように組み替えて，新しいunassigned rowの配列を返す
		int num = static_cast<int>(free_rows.size());
		if(num < 1)
			return 1;
		new_free_rows.clear();
		new_free_rows.reserve(num);
		int aug_row = free_rows[0];
		for(int ind=0 ; ind<num ; ) {
			double min_val_1 = cost_mat_(aug_row-1, 0) - v_(0);
			double min_val_2 = inf_;
			int min_col_1 = 1, min_col_2;
			for(int col=2 ; col<=n_ ; col++) {
				double val = cost_mat_(aug_row-1, col-1) - v_(col-1);
				if(val < min_val_2) {
					if(val >= min_val_1) {
						min_val_2 = val;
						min_col_2 = col;
					}else{
						min_val_2 = min_val_1;
						min_col_2 = min_col_1;
						min_val_1 = val;
						min_col_1 = col;
					}
				}
			}
			///
			int predRow = y_(min_col_1-1);
			if(min_val_2-min_val_1 > epsilon_) {
				v_(min_col_1-1) += min_val_1 - min_val_2;
			}else{
				if(predRow > 0) {
					min_col_1 = min_col_2;
					predRow = y_(min_col_1-1);
				}
			}
			///
			x_(aug_row-1) = min_col_1;
			y_(min_col_1-1) = aug_row;
			///
			if(predRow > 0) {
				if(min_val_2-min_val_1 > epsilon_ && x_(predRow-1) != min_col_1) {
					aug_row = predRow;
				}else{
					new_free_rows.push_back(predRow);
					ind += 1;
					aug_row = free_rows[ind];
				}
				x_(predRow-1) = 0;
			}else{
				ind += 1;
				aug_row = free_rows[ind];
			}
		}
		return 0;
	}

	double cred(const int &row, const int &column) {
		return cost_mat_(row-1, column-1) - u_(row-1) - v_(column-1);
	}

	int Augment(const int &free_row, int &column, const std::deque<int> &ready_deque, const int &minimum_d, const Eigen::ArrayXd &d, const Eigen::ArrayXi &pred) {
		for(unsigned int ind=0 ; ind<ready_deque.size() ; ind++)
			v_(ready_deque[ind]-1) += d(ready_deque[ind]-1) - minimum_d;
		int row = 0;
		while(row != free_row) {
			row = pred(column-1);
			y_(column-1) = row;
			assert( column != x_(row-1) );	// Infinite loop check
			std::swap(column, x_.coeffRef(row-1));
		}
		return 0;
	}

	int Augmentation(std::vector<int> &free_rows) {
		if(free_rows.size() < 1)
			return 1;
		for(unsigned int ind=0 ; ind<free_rows.size() ; ind++) {
			int free_row = free_rows[ind];
			double minimum_d;
			std::deque<int> ready_deque, scan_deque;
			std::vector<int> todo_list;
			ready_deque.clear(); scan_deque.clear(); todo_list.clear();
			Eigen::ArrayXi pred = Eigen::ArrayXi::Constant(n_, free_row);
			Eigen::ArrayXd d = Eigen::ArrayXd::Zero(n_);
			for(int col=1 ; col<=n_ ; col++) {
				todo_list.push_back(col);
				d(col-1) = cost_mat_(free_row-1, col-1) - v_(col-1);
			}
			while(1) {
				int done = 0;
				if(scan_deque.empty()) {
					minimum_d = inf_;
					std::vector<int> deposit;
					std::vector<int>::iterator todo_itr = todo_list.begin();
					while(todo_itr != todo_list.end()) {
						if(d((*todo_itr)-1) <= minimum_d) {
							if(d((*todo_itr)-1) < minimum_d) {
								minimum_d = d((*todo_itr)-1);
								for(unsigned int ind2=0 ; ind2<scan_deque.size() ; ind2++)
									deposit.push_back(scan_deque[ind2]);
								scan_deque.clear();
							}
							scan_deque.push_back((*todo_itr));
							todo_itr = todo_list.erase(todo_itr);
						}else{
							++todo_itr;
						}
					}
					for(unsigned int ind2=0 ; ind2<deposit.size() ; ind2++)
						todo_list.push_back(deposit[ind2]);
					deposit.clear();
					////
					for(unsigned int ind2=0 ; ind2<scan_deque.size() ; ind2++) {
						if(y_(scan_deque[ind2]-1) == 0) {
							Augment(free_row, scan_deque[ind2], ready_deque, minimum_d, d, pred);
							done = 1;
						}
						if(done) break;
					}
					if(done) break;
				}
				int scan_col = scan_deque.front();
				scan_deque.pop_front();
				int prev_row = y_(scan_col-1);
				ready_deque.push_back(scan_col);
				////
				std::vector<int>::iterator todo_itr = todo_list.begin();
				while(todo_itr != todo_list.end()) {
					int erase = 0;
					if(minimum_d + cred(prev_row, *todo_itr) < d((*todo_itr)-1)) {
						d((*todo_itr)-1) = minimum_d + cred(prev_row, *todo_itr);
						pred((*todo_itr)-1) = prev_row;
						if(d((*todo_itr)-1) == minimum_d) {
							if(y_((*todo_itr)-1) == 0) {
								Augment(free_row, (*todo_itr), ready_deque, minimum_d, d, pred);
								done = 1;
							} else {
								scan_deque.push_back(*todo_itr);
								todo_itr = todo_list.erase(todo_itr);
								erase = 1;
							}
							if(done) break;
						}
					}
					if(!erase)
						++todo_itr;
				}
				if(done) break;
			}
		}
		return 0;
	}
};

}
} /* namespace std */
#endif /* LAPJV_HPP_ */
