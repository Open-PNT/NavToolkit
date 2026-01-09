#include <navtk/filtering/containers/EstimateWithCovariance.hpp>

#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

EstimateWithCovariance::EstimateWithCovariance(Vector arg_estimate, Matrix arg_covariance)
    : estimate(std::move(arg_estimate)), covariance(std::move(arg_covariance)) {
	ValidationContext{}
	    .add_matrix(estimate, "estimate")
	    .dim('N', 1)
	    .add_matrix(covariance, "covariance")
	    .dim('N', 'N')
	    .symmetric()
	    .validate();
}

}  // namespace filtering
}  // namespace navtk
