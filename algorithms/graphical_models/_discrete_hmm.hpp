/*
 * discrete_hmm.hpp
 *
 *  Created on: 2012/11/26
 *      Author: takahashi
 *
 *	Coding style: http://www.textdrop.net/google-styleguide-ja/cppguide.xml
 *
 *	Baum-Welch algorithm here is implemented based on "Pattern Recognition and Machine Learning" (Springer)
 *
 *  Reference of HMM: http://ibisforest.org/index.php?%E9%9A%A0%E3%82%8CMarkov%E3%83%A2%E3%83%87%E3%83%AB
 *  Reference of Baum-Welch algorithm: http://ibisforest.org/index.php?Baum-Welch%E3%82%A2%E3%83%AB%E3%82%B4%E3%83%AA%E3%82%BA%E3%83%A0
 *  Referenve of scaling of Baum-Welch algorithm (partly differ from my algorithm): http://unicorn.ike.tottori-u.ac.jp/murakami/doctor/node15.html
 *
 *  To use seriarization, USE_BOOST_SERIALIZATION must be defined and link libboost_serialization-mt, libboost_filesystem-mt, libboost_system-mt
 */

#ifndef TCPP_DISCRETE_HMM_HPP_
#define TCPP_DISCRETE_HMM_HPP_

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#ifdef USING_BOOST_SERIALIZATION
#include <fstream>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/filesystem.hpp>
#include "../../core/convert.hpp"
#endif

namespace tcpp {
namespace prml {

class DiscreteHMM {
public:
	/* constructors and virtual destructor */
	DiscreteHMM(): hidden_state_num_(), observation_num_(), initial_distribution_(), transition_probability_(), output_probability_() {}
	DiscreteHMM( unsigned int hidden_state_num, unsigned int observation_num ):
		hidden_state_num_(hidden_state_num), observation_num_(observation_num),
		initial_distribution_(hidden_state_num), transition_probability_(hidden_state_num, hidden_state_num), output_probability_(hidden_state_num, observation_num) {}
	virtual ~DiscreteHMM() {}

	/* initializer */
	void uniform_initialize() {
		initial_distribution_ = Eigen::ArrayXd::Constant( hidden_state_num_, 1.0 / static_cast<double>(hidden_state_num_) );
		transition_probability_ = Eigen::ArrayXXd::Constant( hidden_state_num_, hidden_state_num_, 1.0 / static_cast<double>(hidden_state_num_) );
		output_probability_ = Eigen::ArrayXXd::Constant( hidden_state_num_, observation_num_, 1.0 / static_cast<double>(observation_num_) );
	}

	void random_initialize() {
		initial_distribution_ = Eigen::ArrayXd::Random( hidden_state_num_ );
		initial_distribution_ = initial_distribution_.abs();
		initial_distribution_ /= initial_distribution_.sum();

		transition_probability_ = Eigen::ArrayXXd::Random( hidden_state_num_, hidden_state_num_ );
		transition_probability_ = transition_probability_.abs();
		for( int row=0; row<transition_probability_.rows(); ++row ) {
			transition_probability_.row(row) /= transition_probability_.row(row).sum();
		}

		output_probability_ = Eigen::ArrayXXd::Random( hidden_state_num_, observation_num_ );
		output_probability_ = output_probability_.abs();
		for( int row=0; row<output_probability_.rows(); ++row) {
			output_probability_.row(row) /= output_probability_.row(row).sum();
		}
	}

	void normal_initialize() {
		initial_distribution_ = Eigen::ArrayXd::Constant( hidden_state_num_, 1.0/static_cast<double>( hidden_state_num_ ) );

		transition_probability_ = Eigen::ArrayXXd::Zero( hidden_state_num_, hidden_state_num_ );
		for( int row=0; row<transition_probability_.rows(); ++row ) {
			for( int col=0; col<transition_probability_.cols(); ++col ) {
				transition_probability_.coeffRef( row, col ) = exp( -pow(col-row, 2) / 2 ) / sqrt(2*M_PI);
			}
			transition_probability_.row(row) /= transition_probability_.row(row).sum();
		}

		output_probability_ = Eigen::ArrayXXd::Constant( hidden_state_num_, observation_num_, 1.0/static_cast<double>( observation_num_ ) );
	}

	/* accumulators and accessors */
	const Eigen::ArrayXd& initial_distribution() const { return initial_distribution_; }
	double initial_distribution( unsigned int state ) const {
		if( state < hidden_state_num_ )
			return initial_distribution_.coeff( state );
		else
			return -1;
	}
	void set_initial_distribution( const Eigen::ArrayXd& initial_distribution ) { initial_distribution_ = initial_distribution; }

	const Eigen::ArrayXXd& transition_probability() const { return transition_probability_; }
	double transition_probability( unsigned int from, unsigned int to ) const {
		if( from < hidden_state_num_ && to < hidden_state_num_ )
			return transition_probability_.coeff( from, to );
		else
			return -1;
	}
	void set_transition_probability( const Eigen::ArrayXXd& transition_probability ) { transition_probability_ = transition_probability; }

	const Eigen::ArrayXXd& output_probability() const { return output_probability_; }
	double output_probability( unsigned int state, unsigned int observation ) const {
		if( state < hidden_state_num_ && observation < observation_num_ )
			return output_probability_.coeff( state, observation );
		else
			return -1;
	}
	void set_output_probability( const Eigen::ArrayXXd& output_probability ) { output_probability_ = output_probability; }

	// parameter estimation using Baum-Welch algorithm
	int EstimateParameters( const std::vector<std::vector<int> >& sample_sequences, int max_iteration, double epsilon ) {
		Eigen::ArrayXd previous_log_likelihoods, updated_log_likelihoods;

		previous_log_likelihoods = Eigen::ArrayXd::Zero( sample_sequences.size() );
		for( unsigned int i=0; i<sample_sequences.size(); ++i ) {
			previous_log_likelihoods.coeffRef(i) = CalcLogLikelihood( sample_sequences[i] );
		}

		int iteration = 0;
		while(1) {
			UpdateParameters( sample_sequences, updated_log_likelihoods );
			Eigen::ArrayXd absdiff_of_likelihoods = (updated_log_likelihoods - previous_log_likelihoods).abs();
			double maxdiff_of_likelihoods = absdiff_of_likelihoods.maxCoeff();
			if( ++iteration > max_iteration || maxdiff_of_likelihoods < epsilon ) {
				break;
			} else {
				previous_log_likelihoods = updated_log_likelihoods;
			}
		}
		return iteration;
	}

	double CalcLogLikelihood( const std::vector<int>& observations ) const {
		double log_likelihood = 0.0;

		std::vector<Eigen::ArrayXd> forward_probabilities;
		std::vector<double> scaling_factors;
		CalcForwardProbabilities( observations, forward_probabilities, scaling_factors );

		for( unsigned int n = 0 ; n < scaling_factors.size() ; ++n )
			log_likelihood += log( scaling_factors[n] );

		return log_likelihood;
	}

private:
	unsigned int hidden_state_num_, observation_num_;
	Eigen::ArrayXd initial_distribution_;
	Eigen::ArrayXXd transition_probability_, output_probability_;
	static const double zero_ = 1E-15;

	/* methods for Baum-Welch algorithm E-step */
	void CalcForwardProbabilities( const std::vector<int>& observations, std::vector<Eigen::ArrayXd>& forward_probabilities, std::vector<double>& scaling_factors ) const {
		unsigned int N = observations.size();
		if( !forward_probabilities.empty() )
			forward_probabilities.clear();
		if( !scaling_factors.empty() )
			scaling_factors.clear();
		forward_probabilities.reserve(N);
		scaling_factors.reserve(N);
		//
		for( unsigned int n = 0; n < N; ++n ) {
			Eigen::ArrayXd forward_probabilities_n = Eigen::ArrayXd::Zero( hidden_state_num_ );
			for( unsigned int state = 0 ; state < hidden_state_num_ ; ++state ) {
				if( n == 0 ) {
					forward_probabilities_n.coeffRef( state ) = initial_distribution_.coeff( state ) * output_probability_.coeff( state, observations[n] );
				} else {
					for( unsigned int previous_state = 0 ; previous_state < hidden_state_num_ ; ++previous_state ) {
						forward_probabilities_n.coeffRef( state ) += forward_probabilities[n-1].coeff( previous_state ) * transition_probability_.coeff( previous_state, state ) * output_probability_.coeff( state, observations[n] );
					}
				}
//				if( forward_probabilities_n.coeff( state ) < zero_ || isnan( forward_probabilities_n.coeff( state ) ) )
//					forward_probabilities_n.coeffRef( state ) = zero_;
			}
			double scaling_factor = forward_probabilities_n.sum();
			forward_probabilities_n /= scaling_factor;
			forward_probabilities.push_back( forward_probabilities_n );
			scaling_factors.push_back( scaling_factor );
		}
	}

	void CalcBackwardProbabilities( const std::vector<int>& observations, std::vector<Eigen::ArrayXd>& backward_probabilities, const std::vector<double>& scaling_factors ) const {
		assert( observations.size() == scaling_factors.size() );
		unsigned int N = observations.size();
		if( !backward_probabilities.empty() )
			backward_probabilities.clear();
		backward_probabilities.resize( N, Eigen::ArrayXd::Zero( hidden_state_num_ ) );
		//
		for( int n = static_cast<int>(N)-1; n >= 0; --n ) {
			Eigen::ArrayXd& backward_probabilities_n = backward_probabilities[n];
			for( unsigned int state = 0 ; state < hidden_state_num_ ; ++state ) {
				if( n == static_cast<int>(N)-1 ) {
					backward_probabilities_n.coeffRef( state ) = 1.0;
				} else {
					for( unsigned int next_state = 0 ; next_state < hidden_state_num_ ; ++next_state ) {
						backward_probabilities_n.coeffRef( state ) +=
								transition_probability_.coeff( state, next_state ) * output_probability_.coeff( next_state, observations[n+1] ) * backward_probabilities[n+1]( next_state );
					}
					backward_probabilities_n.coeffRef( state ) /= scaling_factors[n+1];
				}
//				if( backward_probabilities_n.coeff( state ) < zero_ || isnan( backward_probabilities_n.coeff( state ) ) )
//					backward_probabilities_n.coeffRef( state ) = zero_;
			}
		}
	}

	void CalcLogXi( const std::vector<int>& observations, const std::vector<Eigen::ArrayXd>& forward_probabilities, const std::vector<Eigen::ArrayXd>& backward_probabilities, const std::vector<double>& scaling_factors, std::vector<Eigen::ArrayXXd>& log_xi ) const {
		unsigned int N = observations.size();
		if( !log_xi.empty() )
			log_xi.clear();
		log_xi.reserve(N);
		for( unsigned int n=1; n<N; ++n ) {
			Eigen::ArrayXXd log_xi_n = Eigen::ArrayXXd::Zero( hidden_state_num_, hidden_state_num_ );
			for( unsigned int previous_state = 0 ; previous_state < hidden_state_num_ ; ++previous_state ) {
				for( unsigned int state = 0 ; state < hidden_state_num_ ; ++state ) {
					log_xi_n.coeffRef( previous_state, state ) = log( forward_probabilities[n-1].coeff( previous_state ) ) + log( transition_probability_.coeff( previous_state, state ) ) +
							log( output_probability_.coeff( state, observations[n] ) ) + log( backward_probabilities[n].coeff( state ) ) - log( scaling_factors[n] );
				}
			}
			log_xi.push_back( log_xi_n );
		}
	}

	void CalcGamma( const std::vector<Eigen::ArrayXd>& forward_probabilities, const std::vector<Eigen::ArrayXd>& backward_probabilities, std::vector<Eigen::ArrayXd>& gamma ) const {
		if( !gamma.empty() )
			gamma.clear();
		gamma.reserve( forward_probabilities.size() );
		for( unsigned int n = 0 ; n < forward_probabilities.size() ; ++n ) {
			Eigen::ArrayXd gamma_n = Eigen::ArrayXd::Zero( hidden_state_num_ );
			for( unsigned int state = 0 ; state < hidden_state_num_ ; ++state ) {
				gamma_n.coeffRef( state ) = forward_probabilities[n].coeff( state ) * backward_probabilities[n].coeff( state );
			}
			gamma.push_back( gamma_n );
		}
	}

	/* methods for Baum-Welch algorithm M-step */
	void UpdateInitialDistribution( const std::vector<std::vector<Eigen::ArrayXd> >& gammas ) {
		unsigned int sample_num = gammas.size();
		initial_distribution_ = Eigen::ArrayXd::Zero( hidden_state_num_ );

		for( unsigned int state=0; state<hidden_state_num_; ++state ) {
			for( unsigned int i=0; i<sample_num; ++i ) {
				initial_distribution_.coeffRef( state ) += gammas[i][0].coeff( state );
			}
		}

		initial_distribution_ /= initial_distribution_.sum();
	}

	void UpdateTransitionProbability( const std::vector<std::vector<Eigen::ArrayXXd> >& log_xis, const std::vector<std::vector<Eigen::ArrayXd> >& gammas ) {
		assert( log_xis.size() == gammas.size() );
		unsigned int sample_num = gammas.size();
		transition_probability_ = Eigen::ArrayXXd::Zero( hidden_state_num_, hidden_state_num_ );
		Eigen::ArrayXXd denominators = Eigen::ArrayXXd::Zero( hidden_state_num_, hidden_state_num_ );
		
		for( unsigned int from_state = 0 ; from_state < hidden_state_num_ ; ++from_state ) {
			for( unsigned int to_state = 0 ; to_state < hidden_state_num_ ; ++to_state ) {
				for( unsigned int i=0; i<sample_num; ++i ) {
					for( unsigned int n=0; n<log_xis[i].size(); ++n ) {
						denominators.coeffRef( from_state, to_state ) += gammas[i][n+1].coeff( from_state );
						transition_probability_.coeffRef( from_state, to_state ) += exp( log_xis[i][n].coeff( from_state, to_state ) );
					}
				}
			}
		}
		
		transition_probability_ /= denominators;
	}

	void UpdateOutputProbability( const std::vector<std::vector<int> >& sample_sequences, const std::vector<std::vector<Eigen::ArrayXd> >& gammas ) {
		assert( sample_sequences.size() == gammas.size() );
		unsigned int sample_num = sample_sequences.size();
		output_probability_ = Eigen::ArrayXXd::Zero( hidden_state_num_, observation_num_ );
		Eigen::ArrayXXd denominators = Eigen::ArrayXXd::Zero( hidden_state_num_, observation_num_ );

		for( unsigned int state=0; state<hidden_state_num_; ++state ) {
			for( unsigned int observation=0; observation<observation_num_; ++observation ) {
				for( unsigned int i=0; i<sample_num; ++i ) {
					for( unsigned int n=0; n<sample_sequences[i].size(); ++n ) {
						if( sample_sequences[i][n] == static_cast<int>( observation ) )
							output_probability_.coeffRef( state, observation ) += gammas[i][n].coeff( state );
						denominators.coeffRef( state, observation ) += gammas[i][n].coeff( state );
					}
				}
			}
		}

		output_probability_ /= denominators;
	}

	/* One step of Baum-Welch algorithm */
	double UpdateParameters( const std::vector<std::vector<int> >& sample_sequences, Eigen::ArrayXd& updated_log_likelihoods ) {
		std::vector<std::vector<Eigen::ArrayXXd> > log_xis;
		std::vector<std::vector<Eigen::ArrayXd> > gammas;
		log_xis.reserve( sample_sequences.size() );
		gammas.reserve( sample_sequences.size() );
		for( unsigned int i = 0 ; i < sample_sequences.size() ; ++i ) {
			std::vector<Eigen::ArrayXd> forward_probabilities, backward_probabilities, gamma;
			std::vector<Eigen::ArrayXXd> log_xi;
			std::vector<double> scaling_factors;

			CalcForwardProbabilities( sample_sequences[i], forward_probabilities, scaling_factors );
			CalcBackwardProbabilities( sample_sequences[i], backward_probabilities, scaling_factors );
			CalcLogXi( sample_sequences[i], forward_probabilities, backward_probabilities, scaling_factors, log_xi );
			CalcGamma( forward_probabilities, backward_probabilities, gamma );

			log_xis.push_back( log_xi );
			gammas.push_back( gamma );
		}
		UpdateInitialDistribution( gammas );
		UpdateTransitionProbability( log_xis, gammas );
		UpdateOutputProbability( sample_sequences, gammas );

		double total_log_likelihood = 0.0;
		updated_log_likelihoods = Eigen::ArrayXd::Zero( sample_sequences.size() );
		for( unsigned int i = 0 ; i < sample_sequences.size() ; ++i ) {
			double log_likelihood = CalcLogLikelihood( sample_sequences[i] );
			total_log_likelihood += log_likelihood;
			updated_log_likelihoods.coeffRef(i) = log_likelihood;
		}

		return total_log_likelihood;
	}

#ifdef USING_BOOST_SERIALIZATION
public:
	void Save( const boost::filesystem::path& xml_path ) {
		std::ofstream ofs( xml_path.c_str() );
		boost::archive::xml_oarchive oarchive( ofs );
		oarchive << boost::serialization::make_nvp( "DiscreteHMM", *this );
		ofs.close();
	}

	void Load( const boost::filesystem::path& xml_path ) {
		std::ifstream ifs( xml_path.c_str() );
		boost::archive::xml_iarchive iarchive( ifs );
		iarchive >> boost::serialization::make_nvp( "DiscreteHMM", *this );
		ifs.close();
	}

private:
	/* boost::serialization methods */
	friend class boost::serialization::access;
	BOOST_SERIALIZATION_SPLIT_MEMBER();

	template <class Archive>
	void save(Archive& archive, const unsigned int version) const {
		std::vector<double> initial_distribution_vector;
		std::vector<std::vector<double> > transition_probability_vector, output_probability_vector;
		tcpp::Convert<double, -1>( initial_distribution_, initial_distribution_vector );
		tcpp::Convert<double, -1, -1>( transition_probability_, transition_probability_vector );
		tcpp::Convert<double, -1, -1>( output_probability_, output_probability_vector );

		archive & boost::serialization::make_nvp("hidden_state_num", hidden_state_num_);
		archive & boost::serialization::make_nvp("observation_num", observation_num_);
		archive & boost::serialization::make_nvp("initial_distribution", initial_distribution_vector);
		archive & boost::serialization::make_nvp("transition_probability", transition_probability_vector);
		archive & boost::serialization::make_nvp("output_probability", output_probability_vector);
	}

	template <class Archive>
	void load(Archive& archive, const unsigned int version) {
		std::vector<double> initial_distribution_vector;
		std::vector<std::vector<double> > transition_probability_vector, output_probability_vector;

		archive & boost::serialization::make_nvp("hidden_state_num", hidden_state_num_);
		archive & boost::serialization::make_nvp("observation_num", observation_num_);
		archive & boost::serialization::make_nvp("initial_distribution", initial_distribution_vector);
		archive & boost::serialization::make_nvp("transition_probability", transition_probability_vector);
		archive & boost::serialization::make_nvp("output_probability", output_probability_vector);

		tcpp::Convert<double, -1>( initial_distribution_vector, initial_distribution_ );
		tcpp::Convert<double, -1, -1>( transition_probability_vector, transition_probability_ );
		tcpp::Convert<double, -1, -1>( output_probability_vector, output_probability_ );
	}
#endif
};

} /* namespace prml */
} /* namespace tcpp */

#endif /* TCPP_DISCRETE_HMM_HPP_ */
