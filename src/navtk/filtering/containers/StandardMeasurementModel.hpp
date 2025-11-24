#pragma once

#include <functional>

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A measurement model suitable for use in a StandardFusionEngine. This object is not
 * ordinarily used directly. Instead, a MeasurementProcessor is given to the filter which must
 * produce one of these.
 */
class StandardMeasurementModel {
public:
	/// Function type used as the measurement function \f$h(x)\f$.
	typedef std::function<Vector(const Vector&)> MeasurementFunction;

	/// The vector of measurements, containing N elements.
	Vector z;

	/**
	 * The measurement prediction function (i.e. \f$h(x)\f$ in \f$z=h(x)+v\f$). Accepts the state
	 * vector (xhat) (of size M) and returns the expected measurement vector, of size N.
	 */
	MeasurementFunction h;

	/// The Jacobian of h, a matrix of size NxM, where M is the number of states.
	Matrix H;

	/// The measurement noise covariance matrix, an NxN matrix.
	Matrix R;

	/**
	 * Set fields to the given values using `std::move`. Also validates matrix sizes.
	 *
	 * @param z The value to store in #z.
	 * @param h The value to store in #h.
	 * @param H The value to store in #H.
	 * @param R The value to store in #R.
	 *
	 * @throw std::range_error If `z` is not Nx1, `H` is not NxM, or `R` is not NxN and the error
	 * mode is ErrorMode::DIE.
	 */
	StandardMeasurementModel(Vector z, MeasurementFunction h, Matrix H, Matrix R);

	/**
	 * Set fields to the given values using `std::move`. Also validates matrix sizes. #h is set to
	 * \f$ h(x) = Hx \f$.
	 *
	 * @param z The value to store in #z.
	 * @param H The value to store in #H.
	 * @param R The value to store in #R.
	 *
	 * @throw std::range_error If `z` is not Nx1, `H` is not NxM, or `R` is not NxN and the error
	 * mode is ErrorMode::DIE.
	 */
	StandardMeasurementModel(Vector z, Matrix H, Matrix R);
};

/**
 * An alias.
 */
typedef StandardMeasurementModel MeasurementModel;

}  // namespace filtering
}  // namespace navtk
