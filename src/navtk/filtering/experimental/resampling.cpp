#include <navtk/filtering/experimental/resampling.hpp>

#include <xtensor/core/xmath.hpp>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::zeros;
using xt::cumsum;
using xt::floor;
using xt::sum;

namespace navtk {
namespace filtering {
namespace experimental {

ResamplingResult systematic_resampling(const Vector& weights, const size_t* m_arg) {
	ResamplingResult res;
	size_t N = weights.size();

	size_t M;
	if (m_arg) {
		M = m_arg[0];
	} else {
		M = weights.size();
	}

	std::vector<size_t> new_index(M);
	std::vector<size_t> new_index_count(N);

	if (N <= 0) {
		res.index       = new_index;
		res.index_count = new_index_count;

		return res;
	};

	auto cum_sum = xt::cumsum(weights);

	// ensure that the single particle is not skipped
	double thresh = (N > 2) ? navtk::experimental::rand() / static_cast<double>(M) : -1.0;


	size_t k = 0;
	for (size_t j = 0; j < M; j++) {
		double u_level = (thresh + static_cast<double>(j)) / static_cast<double>(M);
		while (cum_sum(k) < u_level && k < N) {
			k++;
		}
		if (k < N) {
			new_index[j] = k;
			new_index_count[k]++;
		}
	}
	res.index       = new_index;
	res.index_count = new_index_count;

	return res;
}

ResamplingResult residual_resample_with_replacement(const Vector& weights, const size_t* m_arg) {
	// Liu's residual resampling algorithm and Niclas' magic line from Arnaud Doucet and Nando de
	// Freitas
	ResamplingResult res;

	size_t N = weights.size();
	std::vector<size_t> new_index(N);
	std::vector<size_t> init_index(N);
	std::vector<size_t> new_index_count(N);

	size_t M;
	if (m_arg) {
		M = m_arg[0];
	} else {
		M = weights.size();
	}

	if (N == 0 || N != M) {
		log_or_throw("Exception Occurred: Resampling length should be consistent and not empty.");
		res.index       = new_index;
		res.index_count = new_index_count;

		return res;
	}

	Vector weight_residuals = static_cast<double>(N) * weights;
	Vector num_children     = floor(weight_residuals);
	size_t num_residuals    = N - static_cast<size_t>(sum(num_children)(0));

	if (num_residuals != 0) {
		weight_residuals = (weight_residuals - num_children) / static_cast<double>(num_residuals);
		Vector cum_dist  = cumsum(weight_residuals);

		// generate num_residuals uniform RNG
		Vector uniform_values = zeros(num_residuals);

		double cum_prod = 1.;
		for (size_t i = 0; i < num_residuals; i++) {
			double unif_rv =
			    pow(navtk::experimental::rand(), 1. / static_cast<double>(num_residuals - i));
			cum_prod *= unif_rv;
			uniform_values(num_residuals - 1 - i) = cum_prod;
		}

		size_t j = 0;
		for (size_t i = 0; i < num_residuals; i++) {
			while (uniform_values(i) >= cum_dist(j) && j < N) {
				j++;
			}
			num_children(j) += 1;
		}
	}

	// copy the resampled selections
	size_t index = 0;
	for (size_t i = 0; i < N; i++) {
		if (num_children(i) > 0) {
			for (size_t j = index; j < index + num_children(i); j++) {
				new_index[j] = i;
				new_index_count[i]++;
			}
		}
		index += num_children(i);
	}

	res.index       = new_index;
	res.index_count = new_index_count;

	return res;
}

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
