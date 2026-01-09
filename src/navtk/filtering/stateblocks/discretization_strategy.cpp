#include <navtk/filtering/stateblocks/discretization_strategy.hpp>

#include <cmath>

#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using navtk::navutils::calc_van_loan;
using navtk::navutils::discretize_first_order;
using navtk::navutils::discretize_second_order;

namespace navtk {
namespace filtering {
std::pair<Matrix, Matrix> first_order_discretization_strategy(const Matrix& F,
                                                              const Matrix& G,
                                                              const Matrix& Q,
                                                              double dt) {
	if (is_identity(G)) {
		return discretize_first_order(F, Q, dt);
	} else {
		auto q_map = dot(dot(G, Q), transpose(G));
		return discretize_first_order(F, q_map, dt);
	}
}

std::pair<Matrix, Matrix> second_order_discretization_strategy(const Matrix& F,
                                                               const Matrix& G,
                                                               const Matrix& Q,
                                                               double dt) {
	if (is_identity(G)) {
		return discretize_second_order(F, Q, dt);
	}
	{
		auto q_map = dot(dot(G, Q), transpose(G));
		return discretize_second_order(F, q_map, dt);
	}
}

std::pair<Matrix, Matrix> full_order_discretization_strategy(const Matrix& F,
                                                             const Matrix& G,
                                                             const Matrix& Q,
                                                             double dt) {
	return {expm(F * dt), calc_van_loan(F, G, Q, dt)};
}

}  // namespace filtering
}  // namespace navtk
