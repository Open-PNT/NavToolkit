#include <navtk/filtering/containers/StandardMeasurementModel.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

StandardMeasurementModel::StandardMeasurementModel(Vector z,
                                                   MeasurementFunction h,
                                                   Matrix H,
                                                   Matrix R)
    : z(std::move(z)), h(std::move(h)), H(std::move(H)), R(std::move(R)) {

	if (ValidationContext validation{}) {
		validation.add_matrix(this->z, "z")
		    .dim('N', 1)
		    .add_matrix(this->H, "H")
		    .dim('N', 'M')
		    .add_matrix(this->R, "R")
		    .dim('N', 'N')
		    .validate();
	}
}
StandardMeasurementModel::StandardMeasurementModel(Vector z, Matrix H, Matrix R)
    : z(std::move(z)),
      h([=](const Vector& x) { return dot(H, x); }),
      H(std::move(H)),
      R(std::move(R)) {

	if (ValidationContext validation{}) {
		validation.add_matrix(this->z, "z")
		    .dim('N', 1)
		    .add_matrix(this->H, "H")
		    .dim('N', 'M')
		    .add_matrix(this->R, "R")
		    .dim('N', 'N')
		    .validate();
	}
}

}  // namespace filtering
}  // namespace navtk
