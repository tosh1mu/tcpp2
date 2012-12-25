/*
 * combination.hpp
 *
 *  Created on: 2012/11/14
 *      Author: takahashi
 */

#ifndef TCPP_COMBINATION_HPP_
#define TCPP_COMBINATION_HPP_

#include "../core.hpp"

#include <vector>
#include <boost/array.hpp>

namespace tcpp{

template <size_t k>
class Combination {
public:
	Combination(): n_(0), count_(0) {
		for(unsigned int i=0 ; i<k ; ++i)
			out_of_range_[i] = -1;
	}
	Combination(size_t n): n_(n), count_(0) {
		if(!combinations_.empty())
			combinations_.clear();
		boost::array<int, k> combination;
		generate(0, combination);
		for(unsigned int i=0 ; i<k ; ++i)
			out_of_range_[i] = -1;
	}
	virtual ~Combination() {}
	const boost::array<int, k>& operator()() {
		if(count_ < static_cast<int>(combinations_.size())) {
			++count_;
			return combinations_[count_-1];
		}else {
			return out_of_range_;
		}
	}
private:
	size_t n_;
	int count_;
	std::vector<boost::array<int, k> > combinations_;
	boost::array<int, k> out_of_range_;
	void generate(int index, boost::array<int, k>& combination) {
		int min, max;
		if(index > 0) {
			min = combination[index-1] + 1;
		}else{
			min = 0;
		}
		max = static_cast<int>(n_-1) - (static_cast<int>(k-1) - index);
		for(int i=min ; i<=max ; ++i) {
			combination[index] = i;
			if(index == static_cast<int>(k-1)) {
				combinations_.push_back(combination);
			}else{
				generate(index+1, combination);
			}
		}
	}
};

} /* namespace tcpp */

#endif /* TCPP_COMBINATION_HPP_ */
