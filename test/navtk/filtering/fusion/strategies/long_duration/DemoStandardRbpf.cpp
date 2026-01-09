#include "DemoStandardRbpf.hpp"

#include <navtk/factory.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/inspect.hpp>

using std::function;
using std::vector;
using namespace navtk::filtering;
using navtk::eye;
using navtk::num_cols;
using navtk::Size;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::experimental::RbpfStrategy;

namespace detail {

DemoStandardRbpf::DemoStandardRbpf(int test_num) {
	out          = zeros(1, rbpf_test.test_size(test_num));
	outP         = zeros(1, rbpf_test.test_size(test_num));
	truth        = zeros(1, rbpf_test.test_size(test_num));
	measurements = zeros(1, rbpf_test.test_size(test_num) / 10);

	vector<double> tmp_truth        = rbpf_test.truth(test_num);
	vector<double> tmp_measurements = rbpf_test.measurements(test_num);

	for (size_t i = 0; i < tmp_truth.size(); ++i) truth(i) = tmp_truth[i];
	for (size_t i = 0; i < tmp_measurements.size(); ++i) measurements[i] = tmp_measurements[i];

	auto I = eye(2, 2);

	/* Set up the RBPF
	 */
	size_t num_particles = 100;
	RbpfStrategy strategy(num_particles, 0.7, false);
	strategy.on_fusion_engine_state_block_added(2);
	strategy.set_estimate_slice(Vector{0, 2});
	strategy.set_covariance_slice(I);

	vector<size_t> marks{1};

	strategy.set_marked_states(marks);

	for (Size ii = 0; ii < num_cols(out); ++ii) {
		strategy.propagate(StandardDynamicsModel([](Vector it) -> Vector { return it; }, I, I));

		if (!(ii % 10)) {
			Matrix measurement_matrix({{1, 0}});

			Vector measurement({measurements[ii / 10]});

			Matrix covariance({{2}});
			strategy.update(
			    StandardMeasurementModel(measurement,
			                             ([](Vector it) -> Vector { return Vector{it[0]}; }),
			                             measurement_matrix,
			                             covariance));
		}
		out[ii]  = strategy.get_estimate()(0);
		outP[ii] = strategy.get_covariance()(0, 0);
	}
}

}  // namespace detail
